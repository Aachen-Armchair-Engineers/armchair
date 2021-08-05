#include <armchair/nn_pipeline.hpp>
#include "depthai/depthai.hpp"

void DoorDetectionPipeline::initDepthaiDev(std::string nnPath, int width, int height){

    bool syncNN = true;
    auto colorCam = _p.create<dai::node::ColorCamera>();
    auto monoLeft =  _p.create<dai::node::MonoCamera>();
    auto monoRight = _p.create<dai::node::MonoCamera>();
    auto stereo   = _p.create<dai::node::StereoDepth>();
    auto xlinkOut = _p.create<dai::node::XLinkOut>();
    auto xoutDepth = _p.create<dai::node::XLinkOut>();

    auto spatialDetectionNetwork = _p.create<dai::node::MobileNetSpatialDetectionNetwork>();
    auto nnOut = _p.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    nnOut->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    colorCam->setPreviewSize(width, height);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(false);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    colorCam->setFps(40);

    stereo->setConfidenceThreshold(255);
    stereo->setSubpixel(false);

    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    // Link plugins CAM -> NN -> XLINK
    
    //colorCam->preview.link(detectionNetwork->input);
    colorCam->preview.link(spatialDetectionNetwork->input);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);

    //passthrough depth
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    //if(syncNN) detectionNetwork->passthrough.link(xlinkOut->input);
    if(syncNN) spatialDetectionNetwork->passthrough.link(xlinkOut->input);
    else colorCam->preview.link(xlinkOut->input);

    ///detectionNetwork->out.link(nnOut->input);
    spatialDetectionNetwork->out.link(nnOut->input);


    _dev = std::make_unique<dai::Device>(_p);
    _dev->startPipeline();

    _opImageStreams.push_back(_dev->getOutputQueue("preview", 30, false));
    _opImageStreams.push_back(_dev->getOutputQueue("depth", 30, false));
    _opNNetStreams.push_back(_dev->getOutputQueue("detections", 30, false));

}


std::vector<std::shared_ptr<dai::DataOutputQueue>> DoorDetectionPipeline::getExposedImageStreams(){
    return _opImageStreams;
}


std::vector<std::shared_ptr<dai::DataOutputQueue>> DoorDetectionPipeline::getExposedNnetStreams(){
    return _opNNetStreams;
}