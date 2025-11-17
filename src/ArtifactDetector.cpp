#include "artifacts_cvsa/ArtifactDetector.h"

ArtifactDetector::ArtifactDetector(void) : nh_() { 
    this->pub_ = this->nh_.advertise<artifacts_cvsa::artifact_presence>("/cvsa/artifact_presence", 10);
    this->sub_ = this->nh_.subscribe("/neurodata", 1, &ArtifactDetector::on_received_data, this);

    this->buffer_ = new rosneuro::RingBuffer<float>();
    this->has_new_data_ = false;
    this->has_artifact_ = false;
    this->is_configured_ = false;
}
ArtifactDetector::~ArtifactDetector(){
    delete this->buffer_;
}

void ArtifactDetector::run(){
    ros::Rate r(512);
    if(this->is_configured_ == false){
        ROS_ERROR("[ArtifactDetector] ArtifactDetector not configured correctly");
        return;
    }

    while(ros::ok()){
        if(this->has_new_data_){
            ArtifactDetector::ApplyResults res = this->apply();
            this->has_new_data_ = false;
            
            if(res == ArtifactDetector::ApplyResults::Error){
                ROS_ERROR("[ArtifactDetector] Error in ArtifactDetector processing");
                break;
            }else if(res == ArtifactDetector::ApplyResults::BufferNotFull){
                this->set_message();
                this->pub_.publish(this->out_);
                ROS_WARN("[ArtifactDetector] Buffer not full");
                continue;
            }
            this->pub_.publish(this->out_);
            this->has_artifact_ = false;
        }
        ros::spinOnce();
        r.sleep();
    }
}

void ArtifactDetector::set_message(){
    this->out_.header.stamp = ros::Time::now();
    this->out_.seq = this->seq_id_;
    this->out_.has_artifact = this->has_artifact_;
}

ArtifactDetector::ApplyResults ArtifactDetector::apply(void){

    this->buffer_->add(this->data_in_.transpose().cast<float>()); // [samples x channels]
    if(!this->buffer_->isfull()){
        return ArtifactDetector::ApplyResults::BufferNotFull;
    }

    try{
        Eigen::MatrixXf data_buffer = this->buffer_->get(); // [samples x channels]
        int bufferSize, EOG_ch_size;
        this->buffer_->getParam(std::string("size"), bufferSize);
        EOG_ch_size = this->EOG_ch_.size();
        Eigen::MatrixXd dfet = Eigen::MatrixXd::Zero(bufferSize, EOG_ch_size);

        Eigen::MatrixXd data1, eog_data, peaks_data;
        data1 = this->filter_low_EOG_.apply(data_buffer.cast<double>());
        eog_data = this->filter_high_EOG_.apply(data1);

        // EOG
        for(int i = 0; i < EOG_ch_size; i++){
            dfet.col(i) = eog_data.col(this->EOG_ch_[i]-1).cast<double>();
        }
        Eigen::VectorXd heog, veog;
        heog = dfet.col(0) - dfet.col(1); 
        if(EOG_ch_size == 2){
            veog = (dfet.col(0) + dfet.col(1)) / 2.0f; 
        }else if (EOG_ch_size == 3){
            veog = (dfet.col(0) + dfet.col(1)) / 2.0f - dfet.col(2); 
        }

        // peaks
        int nchannels_noEOG = data_buffer.cols() - EOG_ch_size;
        dfet = Eigen::MatrixXd::Zero(bufferSize, nchannels_noEOG);
        peaks_data = this->filter_high_peaks_.apply(data_buffer.cast<double>());
        int j = 0;
        for(int i = 0; i < data_buffer.cols(); i++){
            if(std::find(this->EOG_ch_.begin(), this->EOG_ch_.end(), i+1) != this->EOG_ch_.end()){
                continue; // skip EOG channels
            }
            dfet.col(j) = peaks_data.col(i-1).cast<double>();
            j++;
        }

        // decision message
        if(heog.cwiseAbs().maxCoeff() > this->th_hEOG_ || veog.cwiseAbs().maxCoeff() > this->th_vEOG_ || dfet.cwiseAbs().maxCoeff() > this->th_peaks_){
            this->has_artifact_ = true;
        }
        this->set_message();

        return ArtifactDetector::ApplyResults::Success;

    }catch(std::exception& e){
        ROS_ERROR("[ArtifactDetector] Error in ArtifactDetector processing: %s", e.what());
        return ArtifactDetector::ApplyResults::Error;
    }
}

void ArtifactDetector::on_received_data(const rosneuro_msgs::NeuroFrame &msg){
    this->has_new_data_ = true;
    this->has_artifact_ = false;

    float* ptr_in;
    float* ptr_eog;
    ptr_in = const_cast<float*>(msg.eeg.data.data());
    ptr_eog = const_cast<float*>(msg.exg.data.data());
    
    if(this->modality_ == "online"){ // reminder: if EOG the last channel is mapped in the exg
        this->data_in_ = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, this->nchannels_, this->chunkSize_); // channels x sample
    }else if(this->modality_ == "offline"){
        Eigen::MatrixXf eeg_data = Eigen::Map<rosneuro::DynamicMatrix<float>>(ptr_in, this->nchannels_ - 1, this->chunkSize_); // for the eog
        Eigen::MatrixXf eog_data = Eigen::Map<Eigen::Matrix<float, 1, -1>>(ptr_eog, 1, this->chunkSize_);
        this->data_in_ = Eigen::MatrixXf(this->nchannels_, this->chunkSize_); // channels x sample

        // only the last channel is classified as eog (even if it is wrong, since the eog channel is the 18 in py notation)
        this->data_in_.block(0, 0, this->nchannels_-1, this->chunkSize_) = eeg_data;
        this->data_in_.row(this->nchannels_-1) = eog_data;
    }
    this->seq_id_ = msg.neuroheader.seq;
}

bool ArtifactDetector::configure(void){

    // General information
    if(ros::param::get("~nchannels", this->nchannels_) == false){
        ROS_ERROR("[ArtifactDetector] Missing 'nchannels' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~chunkSize", this->chunkSize_) == false){
        ROS_ERROR("[ArtifactDetector] Missing 'chunkSize' parameter, which is a mandatory parameter");
        return false;
    }
    if(ros::param::get("~modality", this->modality_) == false){
        ROS_ERROR("[ArtifactDetector] Missing 'modality' parameter, which is a mandatory parameter");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_hEOG"), this->th_hEOG_)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param th_hEOG");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_vEOG"), this->th_vEOG_)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param th_vEOG");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("th_peaks"), this->th_peaks_)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param th_peaks");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("EOG_ch"), this->EOG_ch_)) { // write down is important the order!
        ROS_ERROR("[ArtifactDetector] Cannot find param EOG_ch");
        return false;
    }

    // Buffer configuration 
    if(!this->buffer_->configure("RingBufferCfg")){
        ROS_ERROR("[%s] Buffer not configured correctly", this->buffer_->name().c_str());
        return false;
    }

    // Filter parameters
    int filterOrder_EOG, filterOrder_peaks;
    double sampleRate, freq_high_EOG, freq_low_EOG, freq_high_peaks;
    if(!ArtifactDetector::getParam(std::string("sampleRate"), sampleRate)){
        ROS_ERROR("[ArtifactDetector] Missing 'sampleRate' parameter, which is a mandatory parameter");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("freq_high_EOG"), freq_high_EOG)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param freq_high_EOG");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("freq_low_EOG"), freq_low_EOG)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param freq_low_EOG");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("freq_high_peaks"), freq_high_peaks)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param freq_high_peaks");
        return false;
    }

    if (!ArtifactDetector::getParam(std::string("filterOrder_EOG"), filterOrder_EOG)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param filterOrder_EOG");
        return false;
    }
    if (!ArtifactDetector::getParam(std::string("filterOrder_peaks"), filterOrder_peaks)) {
        ROS_ERROR("[ArtifactDetector] Cannot find param filterOrder_peaks");
        return false;
    }

    this->filter_low_EOG_    = rosneuro::Butterworth<double>(rosneuro::ButterType::LowPass,  filterOrder_EOG,  freq_low_EOG, sampleRate);
    this->filter_high_EOG_   = rosneuro::Butterworth<double>(rosneuro::ButterType::HighPass,  filterOrder_EOG,  freq_high_EOG, sampleRate);
    this->filter_high_peaks_ = rosneuro::Butterworth<double>(rosneuro::ButterType::HighPass,  filterOrder_peaks,  freq_high_peaks, sampleRate);
    

    return true;
}

bool ArtifactDetector::configure(const std::string& param_name) {
    bool retval = false;
    XmlRpc::XmlRpcValue config;
    if (!this->nh_.getParam(param_name, config)) {
        ROS_ERROR("Could not find parameter %s on the server, are you sure that it was pushed up correctly?", param_name.c_str());
        return false;
    }

    retval = this->configure(config);
    return retval;
}

bool ArtifactDetector::configure(XmlRpc::XmlRpcValue& config) {
    if (this->is_configured_) {
        ROS_ERROR("ArtifactDetector %s already being reconfigured", this->name_.c_str());
    }
    this->is_configured_ = false;

    bool retval = this->loadConfiguration(config);
    retval = retval && this->configure();
    this->is_configured_ = retval;
    return retval;
}

bool ArtifactDetector::loadConfiguration(XmlRpc::XmlRpcValue& config) {
    if(config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR("An ArtifactDetector configuration must be a map with fields name, and params");
        return false;
    }

    if (!setName(config)) {
        return false;
    }

    if(config.hasMember("params")) {
        XmlRpc::XmlRpcValue params = config["params"];
        if(params.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR("params must be a map");
            return false;
        } else {
            for(XmlRpc::XmlRpcValue::iterator it = params.begin(); it != params.end(); ++it) {
                ROS_DEBUG("Loading param %s", it->first.c_str());
                this->params_[it->first] = it->second;
            }
        }
    }
    return true;
}

bool ArtifactDetector::setName(XmlRpc::XmlRpcValue& config) {
    if(!config.hasMember("name")) {
        ROS_ERROR("ArtifactDetector didn't have name defined, this is required");
        return false;
    }

    this->name_ = std::string(config["name"]);
    ROS_DEBUG("Configuring ArtifactDetector with name %s", this->name_.c_str());
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::string& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
        return false;
    }

    auto tmp = it->second;
    value = std::string(tmp);
    return true;
}


bool ArtifactDetector::getParam(const std::string& name, bool& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        return false;
    }

    auto tmp = it->second;
    value = (bool)(tmp);
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, double& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeDouble && it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        return false;
    }

    auto tmp = it->second;
    value = it->second.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(tmp) : (double)(tmp);
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, int& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeInt) {
        return false;
    }

    auto tmp = it->second;
    value = tmp;
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, unsigned int& value) const {
    int signed_value;
    if (!this->getParam(name, signed_value))
        return false;
    if (signed_value < 0)
        return false;
    value = signed_value;
    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<double>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue double_array = it->second;

    for (auto i = 0; i < double_array.size(); ++i){
        if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            return false;
        }

        double double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (double)(double_array[i]);
        value.push_back(double_value);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<int>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue double_array = it->second;

    for (auto i = 0; i < double_array.size(); ++i){
        if(double_array[i].getType() != XmlRpc::XmlRpcValue::TypeDouble && double_array[i].getType() != XmlRpc::XmlRpcValue::TypeInt) {
            return false;
        }

        int double_value = double_array[i].getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(double_array[i]) : (int)(double_array[i]);
        value.push_back(double_value);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, std::vector<std::string>& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    value.clear();

    if(it->second.getType() != XmlRpc::XmlRpcValue::TypeArray)
        return false;

    XmlRpc::XmlRpcValue string_array = it->second;

    for (auto i = 0; i < string_array.size(); ++i) {
        if(string_array[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
            return false;
        }

        value.push_back(string_array[i]);
    }

    return true;
}

bool ArtifactDetector::getParam(const std::string& name, XmlRpc::XmlRpcValue& value) const {
    auto it = this->params_.find(name);
    if (it == this->params_.end()) {
        return false;
    }

    auto tmp = it->second;
    value = tmp;
    return true;
}
