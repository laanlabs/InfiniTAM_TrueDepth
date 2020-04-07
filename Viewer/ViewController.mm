//
//  ViewController.m
//  InfiniTAM
//
//  Created by Victor Adrian Prisacariu on 29/10/2014.
//  Copyright (c) 2014 Victor Adrian Prisacariu. All rights reserved.
//

#import "ViewController.h"

#include "Engine/ImageSourceEngine.h"
#include "Engine/IMUSourceEngine.h"

#include "ITMLib/ITMLib.h"
#include "ORUtils/MetalContext.h"

#include "Utils/FileUtils.h"




using namespace InfiniTAM::Engine;

@interface ViewController()

@property (nonatomic, strong) dispatch_queue_t renderingQueue;
@property (nonatomic, strong) MetalContext *context;

@end

@implementation ViewController
{
    CGColorSpaceRef rgbSpace;
    Vector2i imageSize;
    ITMUChar4Image *result;
    
    ImageSourceEngine *imageSource;
    IMUSourceEngine *imuSource;
    ITMLibSettings *internalSettings;
    ITMMainEngine *mainEngine;
    
    ITMIMUMeasurement *imuMeasurement;
    
    ITMUChar4Image *inputRGBImage; ITMShortImage *inputRawDepthImage;
    
    STSensorController *_sensorController;
    
    NSLock * frameLock;
    
    bool isDone;
    bool fullProcess;
    bool isRecording;
    bool usingSensor;
    
    int currentFrameNo;
    
    int depthFrameIndex;
    
    
    bool useStructureSensor;
    
    NSTimeInterval totalProcessingTime;
    int totalProcessedFrames;
    
    char documentsPath[1000], *docsPath;
    NSString * docsPathString;
    
    // true depth stuff:
    bool authWasSuccessful;
    AVCaptureSession * session;
    bool isSessionRunning;
    dispatch_queue_t sessionQueue;
    AVCaptureDeviceInput * videoDeviceInput;
    dispatch_queue_t dataOutputQueue;
    
    AVCaptureVideoDataOutput * videoDataOutput;
    AVCaptureDepthDataOutput * depthDataOutput;
    AVCaptureDataOutputSynchronizer * outputSynchronizer;
    
    AVCaptureDeviceDiscoverySession * videoDeviceDiscoverySession;
    
    
}

- (void) viewDidLoad
{
    [super viewDidLoad];
    
    useStructureSensor = false;
    authWasSuccessful = false;
    
    frameLock = [NSLock new];
    //QOS_CLASS_USER_INTERACTIVE
    //dispatch_queue_attr_make_with_qos_class(<#dispatch_queue_attr_t  _Nullable attr#>, <#dispatch_qos_class_t qos_class#>, <#int relative_priority#>)
    
    //self.renderingQueue = dispatch_queue_create("rendering", DISPATCH_QUEUE_SERIAL);
    
    dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE,1);
    self.renderingQueue = dispatch_queue_create("rendering", attr);
    
    if ( useStructureSensor ) {
        _sensorController = [STSensorController sharedController];
        _sensorController.delegate = self;
    } else {
        
    }
    
    _motionManager = [[CMMotionManager alloc]init];
    _motionManager.deviceMotionUpdateInterval = 1.0f / 60.0f;
    
    totalProcessingTime = 0;
    totalProcessedFrames = 0;
    
    depthFrameIndex = 0;
    
    //dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(1.0 * NSEC_PER_SEC)), dispatch_get_main_queue(), ^{
    //});
    
    if ( useStructureSensor ) {
        [self setupAppStructure];
    } else {
        [self setupAppTrueDepth];
    }
    
}

- (void) viewDidAppear:(BOOL)animated
{
    dispatch_async(sessionQueue, ^{
        
        
        if ( authWasSuccessful ) {
            
            [session startRunning];
            isSessionRunning = session.isRunning;
            
        } else {
            NSLog(@" Auth or camera setup failed!");
        }
        
        
    });
    
    
}

- (void)viewWillDisappear:(BOOL)animated {
    
    dispatch_async(sessionQueue, ^{
        if ( authWasSuccessful ) {
            [session stopRunning];
            isSessionRunning = false;
        }
    });
    
    [super viewWillDisappear:animated];
}

- (void) didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (void) setupAppTrueDepth {
    
    session = [AVCaptureSession new];
    
    
    auto attr = dispatch_queue_attr_make_with_autorelease_frequency(NULL, DISPATCH_AUTORELEASE_FREQUENCY_WORK_ITEM);
    
    sessionQueue = dispatch_queue_create("session queue", attr);
    dataOutputQueue = dispatch_queue_create("video data queue", attr);

    videoDataOutput = [AVCaptureVideoDataOutput new];
    depthDataOutput = [AVCaptureDepthDataOutput new];
    
    authWasSuccessful = false;
    
    videoDeviceDiscoverySession = [AVCaptureDeviceDiscoverySession discoverySessionWithDeviceTypes:@[AVCaptureDeviceTypeBuiltInTrueDepthCamera]
                                                                                         mediaType:AVMediaTypeVideo position:AVCaptureDevicePositionFront];
    
    
    //dataOutputQueue = DispatchQueue(label: "video data queue", qos: .userInitiated, attributes: [], autoreleaseFrequency: .workItem)
    // sessionQueue = DispatchQueue(label: "session queue", attributes: [], autoreleaseFrequency: .workItem)
    
    
    auto status = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    
    if ( status != AVAuthorizationStatusAuthorized) {
        
        dispatch_suspend(sessionQueue);
        
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo completionHandler:^(BOOL granted) {
            authWasSuccessful = granted;
            if ( granted ) {
                dispatch_resume(sessionQueue);
            }
        }];
        
    } else {
        
        authWasSuccessful = true;
        
        // OK
        
    }
    
    dispatch_async(sessionQueue, ^{
        [self configureSession];
    });
    
    
    
    
}

-(void) configureSession {
    
    if ( !authWasSuccessful ) { return; }
    
    auto defaultVideoDevice = [videoDeviceDiscoverySession devices].firstObject;
    
    if ( defaultVideoDevice == NULL ) {
        NSLog(@"No video device found!");
        return;
    }
    
    NSError * error = NULL;
    videoDeviceInput = [AVCaptureDeviceInput deviceInputWithDevice:defaultVideoDevice error:&error];
    if ( error != NULL ) {
        NSLog(@"Error with video device!");
        return;
    }
    
    [session beginConfiguration];
    session.sessionPreset = AVCaptureSessionPreset640x480;
    [session addInput:videoDeviceInput];
    
    //session.addOutput(videoDataOutput)
    //videoDataOutput.videoSettings = [kCVPixelBufferPixelFormatTypeKey as String: Int(kCVPixelFormatType_32BGRA)]
    [session addOutput:videoDataOutput];
    
    
    
    auto settings = [NSDictionary dictionaryWithObject:[NSNumber numberWithInt:kCVPixelFormatType_32BGRA]
                                                forKey:(id)kCVPixelBufferPixelFormatTypeKey];
    
    videoDataOutput.videoSettings = settings;
    
    
    // Depth
    [session addOutput:depthDataOutput];
    [depthDataOutput setFilteringEnabled:false];
    
    auto connection = [depthDataOutput connectionWithMediaType:AVMediaTypeDepthData];
    [connection setEnabled:true];
    
    
    
    auto formats = defaultVideoDevice.activeFormat.supportedDepthDataFormats;
    int bestFormatIdx=-1;
    int idx = 0;
    for (AVCaptureDeviceFormat* format in formats) {
        
        auto dims = CMVideoFormatDescriptionGetDimensions(format.formatDescription);
        
        if ( CMFormatDescriptionGetMediaSubType(format.formatDescription) == kCVPixelFormatType_DepthFloat16 &&
             dims.width == 640 ) {
            bestFormatIdx = idx;
            NSLog(@"Got good depth format: %i ", bestFormatIdx);
        }
        
        idx ++;
    }
    
    if ( bestFormatIdx != -1 ) {
        
        [defaultVideoDevice lockForConfiguration:NULL];
        [defaultVideoDevice setActiveDepthDataFormat:formats[bestFormatIdx]];
        [defaultVideoDevice unlockForConfiguration];
        
    } else {
        NSLog(@"Could not find good depth format!");
        return;
    }
    
    outputSynchronizer = [[AVCaptureDataOutputSynchronizer alloc] initWithDataOutputs:@[videoDataOutput, depthDataOutput]];
    [outputSynchronizer setDelegate:self queue:dataOutputQueue];
    
    [session commitConfiguration];
    
    
    // ------------------------------------------------------
    
    isDone = false;
    fullProcess = false;
    isRecording = false;
    
    currentFrameNo = 0;
    
    self.context = [MetalContext instance];
    
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    docsPathString = [dirPaths objectAtIndex:0];
    docsPath = (char*)[docsPathString cStringUsingEncoding:[NSString defaultCStringEncoding]];
    
    memcpy(documentsPath, docsPath, strlen(docsPath));
    
    //
    
    fullProcess = false;

    [_motionManager startDeviceMotionUpdates];

    imuMeasurement = new ITMIMUMeasurement();

    
    const char *calibFile = [[[NSBundle mainBundle]pathForResource:@"calib_truedepth" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
    
    imageSource = new CalibSource(calibFile, Vector2i(320, 240), 0.5f);

    NSLog(@"RGBA image size: %i ", imageSource->getRGBImageSize().width );
    
    inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
    inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);

    usingSensor = true;


    imageSize = imageSource->getDepthImageSize();
    result = new ITMUChar4Image(imageSize, false);
    rgbSpace = CGColorSpaceCreateDeviceRGB();

    internalSettings = new ITMLibSettings();

    //internalSettings->trackerType = internalSettings->TRACKER_ICP;

//    internalSettings->noHierarchyLevels = 5;
//    internalSettings->depthTrackerICPThreshold = 0.02;
//    internalSettings->depthTrackerTerminationThreshold = 0.001;

    //internalSettings->
    
    NSLog(@" Tracker type: %i ", internalSettings->trackerType );

    NSLog(@" depthTrackerICPThreshold:  %5.2f", internalSettings->depthTrackerICPThreshold );

    NSLog(@" depthTrackerTerminationThreshold:  %f", internalSettings->depthTrackerTerminationThreshold );

    NSLog(@" noHierarchyLevels:  %5i", internalSettings->noHierarchyLevels );
    NSLog(@" noICPRunTillLevel:  %5i", internalSettings->noICPRunTillLevel );

    //    Tracker type: 1
    //    depthTrackerICPThreshold:   0.01
    //    depthTrackerTerminationThreshold:  0.001000
    //    noHierarchyLevels:      5
    //    noICPRunTillLevel:      0




    //    TRACKER_ICP,
    //    //! Identifies a tracker based on depth image (Ren et al, 2012)
    //    TRACKER_REN, - slow
    //    //! Identifies a tracker based on depth image and IMU measurement
    //    TRACKER_IMU, -
    //    //! Identifies a tracker that use weighted ICP only on depth image
    //    TRACKER_WICP - crash

    mainEngine = new ITMMainEngine( internalSettings,
                                    &imageSource->calib,
                                    imageSource->getRGBImageSize(),
                                    imageSource->getDepthImageSize()
                                  );

    isDone = true;


    
}


- (void) setupAppStructure
{
    isDone = false;
    fullProcess = false;
    isRecording = false;
    
    currentFrameNo = 0;
    
    self.context = [MetalContext instance];
    
    NSArray *dirPaths = NSSearchPathForDirectoriesInDomains(NSDocumentDirectory, NSUserDomainMask, YES);
    docsPath = (char*)[[dirPaths objectAtIndex:0]cStringUsingEncoding:[NSString defaultCStringEncoding]];
    memcpy(documentsPath, docsPath, strlen(docsPath));
    
    NSError *error;
    NSString *dataPath = [[dirPaths objectAtIndex:0] stringByAppendingPathComponent:@"/Output"];
    if (![[NSFileManager defaultManager] fileExistsAtPath:dataPath])
        [[NSFileManager defaultManager] createDirectoryAtPath:dataPath withIntermediateDirectories:NO attributes:nil error:&error];
    
    STSensorControllerInitStatus resultSensor = [_sensorController initializeSensorConnection];
    
    BOOL didSucceed = (resultSensor == STSensorControllerInitStatusSuccess || resultSensor == STSensorControllerInitStatusAlreadyInitialized);
    
    if (!didSucceed)
    {
        
        NSLog(@"NO SENSOR!!~");
        
        char calibFile[2000];
        sprintf(calibFile, "%s/Teddy/calib.txt", documentsPath);
        
        fullProcess = true;
        
//        char imageSource_part1[2000], imageSource_part2[2000];
//        sprintf(imageSource_part1, "%s/Teddy/Frames/%%04i.ppm", documentsPath);
//        sprintf(imageSource_part2, "%s/Teddy/Frames/%%04i.pgm", documentsPath);

        //TODO deallocate somewhere
//        imageSource = new ImageFileReader(calibFile, imageSource_part1, imageSource_part2);

        
        char imageSource_part1[2000], imageSource_part2[2000], imageSource_part3[2000];
        sprintf(imageSource_part1, "%s/CAsmall/Frames/img_%%08d.ppm", documentsPath);
        sprintf(imageSource_part2, "%s/CAsmall/Frames/img_%%08d.irw", documentsPath);
        sprintf(imageSource_part3, "%s/CAsmall/Frames/imu_%%08d.txt", documentsPath);
        
//        TODO deallocate somewhere
        imageSource = new RawFileReader(calibFile, imageSource_part1, imageSource_part2, Vector2i(320, 240), 0.5f);
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        imuSource = new IMUSourceEngine(imageSource_part3);
        
        [_tbOut setText:@"from file"];
        
        usingSensor = false;
        imuMeasurement = new ITMIMUMeasurement();
    }
    else
    {
        fullProcess = false;
        
        [_motionManager startDeviceMotionUpdates];
        
        imuMeasurement = new ITMIMUMeasurement();
        
        STStreamConfig streamConfig = STStreamConfigDepth320x240;
        
        NSError* error = nil;
        BOOL optionsAreValid = [_sensorController startStreamingWithOptions:@{kSTStreamConfigKey : @(streamConfig),
                                                                              kSTFrameSyncConfigKey : @(STFrameSyncOff)} error:&error];
        if (!optionsAreValid)
        {
            NSString *string = [NSString stringWithFormat:@"Error during streaming start: %s", [[error localizedDescription] UTF8String]];
            [_tbOut setText:@"from camera"];
            return;
        }
        
        const char *calibFile = [[[NSBundle mainBundle]pathForResource:@"calib3" ofType:@"txt"] cStringUsingEncoding:[NSString defaultCStringEncoding]];
        imageSource = new CalibSource(calibFile, Vector2i(320, 240), 0.5f);

        if (error != nil) [_tbOut setText:@"from camera -- errors"];
        else [_tbOut setText:@"from camera"];
        
        inputRGBImage = new ITMUChar4Image(imageSource->getRGBImageSize(), true, false);
        inputRawDepthImage = new ITMShortImage(imageSource->getDepthImageSize(), true, false);
        
        usingSensor = true;
    }
    
    imageSize = imageSource->getDepthImageSize();
    result = new ITMUChar4Image(imageSize, false);
    rgbSpace = CGColorSpaceCreateDeviceRGB();
    
    internalSettings = new ITMLibSettings();
    
    internalSettings->trackerType = internalSettings->TRACKER_ICP;
    
    internalSettings->noHierarchyLevels = 5;
    internalSettings->depthTrackerICPThreshold = 0.02;
    internalSettings->depthTrackerTerminationThreshold = 0.001;
    
    //internalSettings->
    NSLog(@" Tracker type: %i ", internalSettings->trackerType );
    
    NSLog(@" depthTrackerICPThreshold:  %5.2f", internalSettings->depthTrackerICPThreshold );
    
    NSLog(@" depthTrackerTerminationThreshold:  %f", internalSettings->depthTrackerTerminationThreshold );
    
    NSLog(@" noHierarchyLevels:  %5i", internalSettings->noHierarchyLevels );
    NSLog(@" noICPRunTillLevel:  %5i", internalSettings->noICPRunTillLevel );
    
//    Tracker type: 1
//    depthTrackerICPThreshold:   0.01
//    depthTrackerTerminationThreshold:  0.001000
//    noHierarchyLevels:      5
//    noICPRunTillLevel:      0
    
    
    
    
//    TRACKER_ICP,
//    //! Identifies a tracker based on depth image (Ren et al, 2012)
//    TRACKER_REN, - slow
//    //! Identifies a tracker based on depth image and IMU measurement
//    TRACKER_IMU, - 
//    //! Identifies a tracker that use weighted ICP only on depth image
//    TRACKER_WICP - crash
    
    mainEngine = new ITMMainEngine(internalSettings, &imageSource->calib, imageSource->getRGBImageSize(),
                                   imageSource->getDepthImageSize());
    
    isDone = true;
}

- (IBAction)bProcessOne_clicked:(id)sender
{
    /*
    if (usingSensor)
    {
        isRecording = !isRecording;
        return;
    }
    
    if (!imageSource->hasMoreImages()) return;
    
    imageSource->getImages(inputRGBImage, inputRawDepthImage);
    
    dispatch_async(self.renderingQueue, ^{
        [self updateImage];
    });
     */
    
    if ( !fullProcess ) {
        
        NSString * objname = [NSString stringWithFormat:@"object_%05d.stl", arc4random() % 1234];
        
        NSString * outPath = [docsPathString stringByAppendingPathComponent:objname];
        mainEngine->SaveSceneToMesh( [outPath cStringUsingEncoding:NSUTF8StringEncoding] );
        NSLog(@"Saved mesh: %@", outPath );
        
    }
}

- (IBAction)bProcessCont_clicked:(id)sender
{
    
    
    if (usingSensor)
    {
        
        if ( !fullProcess ) {
            
            //mainEngine->turnOnMainProcessing();
            
            fullProcess = true;
            
        } else {
            
            fullProcess = false;
            //mainEngine->turnOffMainProcessing();
            
        }
        
    }
    
    
}

- (void) updateImage
{
    
    
    
    //if ( !fullProcess ) { return; }
    
    if ( fullProcess ) {
        mainEngine->turnOnMainProcessing();
    } else {
        mainEngine->turnOffMainProcessing();
    }
        
    NSDate *timerStart = [NSDate date];
    
    if (imuMeasurement != NULL) mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, imuMeasurement);
    else
    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
    
    
    
    NSDate *timerStop = [NSDate date];
    NSTimeInterval executionTime = [timerStop timeIntervalSinceDate:timerStart];
    
    if (fullProcess)
    {
        totalProcessedFrames++;
        totalProcessingTime += executionTime;
    }
    
    if (fullProcess) mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_SCENERAYCAST);
    else mainEngine->GetImage(result, ITMMainEngine::InfiniTAM_IMAGE_ORIGINAL_DEPTH);
    
    CGContextRef cgContext = CGBitmapContextCreate(result->GetData(MEMORYDEVICE_CPU), imageSize.x, imageSize.y, 8,
                                                   4 * imageSize.x, rgbSpace, kCGImageAlphaNoneSkipLast);
    CGImageRef cgImageRef = CGBitmapContextCreateImage(cgContext);
    
    dispatch_sync(dispatch_get_main_queue(), ^{
        self.renderView.layer.contents = (__bridge id)cgImageRef;
        
        NSString *theValue = [NSString stringWithFormat:@"%5.4lf", totalProcessingTime / totalProcessedFrames];
        [self.tbOut setText:theValue];
    });

    CGImageRelease(cgImageRef);
    CGContextRelease(cgContext);
}

// MARK: - True Depth




// MARK: - Structure Sensor

- (void)sensorDidDisconnect
{
    [self.tbOut setText:@"disconnected "];
}

- (void)sensorDidConnect
{
}

- (void)sensorDidLeaveLowPowerMode
{
}

- (void)sensorBatteryNeedsCharging
{
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    [self.tbOut setText:@"stopped streaming"];
}

-(void) sensorDidOutputSynchronizedDepthFrame:(STDepthFrame *)depthFrame andColorBuffer:(CMSampleBufferRef)sampleBuffer
{
    [self.tbOut setText:@"got frame c"];
}



- (void)sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    depthFrameIndex++;
    
    //if ( depthFrameIndex % 2 != 0 ) { return; }
    
    
    if (isDone)
    {
        isDone = false;
        
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        
    
        //[frameLock lock];
        
        memcpy(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), [depthFrame shiftData], imageSize.x * imageSize.y * sizeof(short));
        
//        uint16_t * ptr = (uint16_t *)inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
//        int total_pixels = imageSize.x * imageSize.y;
//        float * depth_mm = depthFrame.depthInMillimeters;
//        for ( int j = 0; j < total_pixels; j++ ) {
//            float D = depth_mm[j];
//            if ( D < 0 ) { D = 0.0; }
//            if ( D >= 65536 ) { D = 65536-1; }
//            ptr[j] = D;
//        }
        
        //[frameLock unlock];
        
        dispatch_async(self.renderingQueue, ^{
            
            [self updateImage];
            
            isDone = true;
        });
    }
}


- (void)OLD______sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    if (isDone)
    {
        isDone = false;
        
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        
    
        [frameLock lock];
        
        memcpy(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), [depthFrame shiftData], imageSize.x * imageSize.y * sizeof(short));
        
//        uint16_t * ptr = (uint16_t *)inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
//        int total_pixels = imageSize.x * imageSize.y;
//        float * depth_mm = depthFrame.depthInMillimeters;
//        for ( int j = 0; j < total_pixels; j++ ) {
//            float D = depth_mm[j];
//            if ( D < 0 ) { D = 0.0; }
//            if ( D >= 65536 ) { D = 65536-1; }
//            ptr[j] = D;
//        }
        
        [frameLock unlock];
        
        dispatch_async(self.renderingQueue, ^{
            if (isRecording)
            {
                FILE *f; char fileName[2000];
                
                sprintf(fileName, "%s/Output/img_%08d.irw", documentsPath, currentFrameNo);
                f = fopen(fileName, "wb+");
                fwrite(inputRawDepthImage->GetData(MEMORYDEVICE_CPU), imageSize.x * imageSize.y * sizeof(short), 1, f);
                fclose(f);
                
                sprintf(fileName, "%s/Output/imu_%08d.txt", documentsPath, currentFrameNo);
                f = fopen(fileName, "w+");
                fprintf(f, "%f %f %f %f %f %f %f %f %f",
                        rotationMatrix.m11, rotationMatrix.m12, rotationMatrix.m13,
                        rotationMatrix.m21, rotationMatrix.m22, rotationMatrix.m23,
                        rotationMatrix.m31, rotationMatrix.m32, rotationMatrix.m33);
                
                fclose(f);
                
                currentFrameNo++;
            }
            
            [self updateImage];
            
            isDone = true;
        });
    }
}

-(void) updateRGBImage:(CVPixelBufferRef)pixelBuffer {
    
    CVPixelBufferLockBaseAddress(pixelBuffer, NULL);
    
    // RGBA 320x240
    uint8_t * ptr = (uint8_t *)inputRGBImage->GetData(MEMORYDEVICE_CPU);
    
    const uint8_t* pixelData = (const uint8_t*)CVPixelBufferGetBaseAddress(pixelBuffer);
    
    int bpr = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    
    if ( bpr != 640*4 ) {
        NSLog(@"bpr pixel buffer: %i ", bpr );
        return;
    }
    
    int size = (int)inputRGBImage->dataSize;
    
    // 76800 = 320x240
    if ( size != 320*240 ) {
        NSLog(@"wrong image size!  %i ", size);
        return;
    }
    
    
    
    auto dims = imageSource->getRGBImageSize();
    
    int scale = 2;  // 640 / 320
    
    int w = dims.width;
    int h = dims.height;
    
    for ( int y = 0; y < h; y++ ) {
        for ( int x = 0; x < w; x++ ) {
            
            const int y2 = y;
            
            int idx = y2 * scale * w * 4 + x * scale * 4;
            
            int r = pixelData[idx];
            int g = pixelData[idx+1];
            int b = pixelData[idx+2];
            //int a = pixelData[idx+3];
            
            int out_idx = y2 * w * 4 + x * 4;
            
            ptr[out_idx] = r;
            ptr[out_idx+1] = g;
            ptr[out_idx+2] = b;
            ptr[out_idx+3] = 255;
            
            
        }
    }
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, NULL);
    
    
}

static int frameIndex = 0;

- (void)dataOutputSynchronizer:(nonnull AVCaptureDataOutputSynchronizer *)synchronizer
        didOutputSynchronizedDataCollection:(nonnull AVCaptureSynchronizedDataCollection *)synchronizedDataCollection {
    
    frameIndex++;
    //if ( frameIndex % 2 == 0) { return; }
    
    if (isDone)
    {
        isDone = false;
        
        CMRotationMatrix rotationMatrix = self.motionManager.deviceMotion.attitude.rotationMatrix;
        
        if (imuMeasurement != NULL)
        {
            imuMeasurement->R.m00 = rotationMatrix.m11; imuMeasurement->R.m01 = rotationMatrix.m12; imuMeasurement->R.m02 = rotationMatrix.m13;
            imuMeasurement->R.m10 = rotationMatrix.m21; imuMeasurement->R.m11 = rotationMatrix.m22; imuMeasurement->R.m12 = rotationMatrix.m23;
            imuMeasurement->R.m20 = rotationMatrix.m31; imuMeasurement->R.m21 = rotationMatrix.m32; imuMeasurement->R.m22 = rotationMatrix.m33;
        }
        
    
        
        AVCaptureSynchronizedDepthData* syncedDepthData =
            (AVCaptureSynchronizedDepthData*)[synchronizedDataCollection synchronizedDataForCaptureOutput:depthDataOutput];
        
        AVCaptureSynchronizedSampleBufferData * syncedVideoData =
            (AVCaptureSynchronizedSampleBufferData *)[synchronizedDataCollection synchronizedDataForCaptureOutput:videoDataOutput];
        
        if ( syncedDepthData.depthDataWasDropped || syncedVideoData.sampleBufferWasDropped ) {
            return;
        }

        auto depthData = syncedDepthData.depthData;
        
        auto sampleBuffer = syncedVideoData.sampleBuffer;
        
        auto pixelBuffer = CMSampleBufferGetImageBuffer(sampleBuffer);
        
        //[self updateRGBImage: pixelBuffer];
        
        ///
        auto depthBuffer = depthData.depthDataMap;
        
        matrix_float3x3 intrinsics = depthData.cameraCalibrationData.intrinsicMatrix;
        CGSize referenceDimensions = depthData.cameraCalibrationData.intrinsicMatrixReferenceDimensions;
        
        float ratio = referenceDimensions.width / CVPixelBufferGetWidth(depthBuffer);
        intrinsics.columns[0][0] /= ratio;
        intrinsics.columns[1][1] /= ratio;
        intrinsics.columns[2][0] /= ratio;
        intrinsics.columns[2][1] /= ratio;
//
//        NSLog(@"K:  %5.2f %5.2f %5.2f %5.2f ", intrinsics.columns[0][0],
//                                               intrinsics.columns[1][1],
//                                               intrinsics.columns[2][0],
//                                               intrinsics.columns[2][1]);
        
        //float16_t
        size_t width = CVPixelBufferGetWidth(depthBuffer);
        size_t height = CVPixelBufferGetHeight(depthBuffer);
        size_t stride = CVPixelBufferGetBytesPerRow(depthBuffer);
        
        //NSLog(@" depth frame: %i x %i ", (int)width, (int)height);
        
        size_t pixelBpr = CVPixelBufferGetBytesPerRow(pixelBuffer);
        
        CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        
        const uint8_t* baseAddress = (const uint8_t*)CVPixelBufferGetBaseAddress(depthBuffer);
        //const uint8_t* basePixel = (const uint8_t*)CVPixelBufferGetBaseAddress(pixelBuffer);
        
        //size_t numPoints = 0;
        
        float minDepth = 0.01;
        float maxDepth = 1.0; // 2.0
        
        
        uint16_t * ptr = (uint16_t *)inputRawDepthImage->GetData(MEMORYDEVICE_CPU);
        
        
        // 640 --> 320
        const int step = 2;
        
        for (size_t y = 0; y < height; y+=step) {
            
            const __fp16* data = (const __fp16*)(baseAddress + y * stride);
            const float py = y;
            
            
            for (size_t x = 0; x < width; x+=step, data+=step) {
                
                __fp16 depth2 = *data;
                float depth = depth2;
                
                const float px = x;
                
                if ( isnan(depth) ) { depth = 0; }
                if ( depth > maxDepth ) { depth = 0; }
                if ( depth < 0 ) { depth = 0; }
                
                uint16_t depth_mm = depth * 1000;
                
                int ptr_idx = ( (height-y)/step) * imageSize.x + (x/step);
                ptr[ptr_idx] = depth_mm;
                
            }
        }
        
        CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
        CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
        
        dispatch_async(self.renderingQueue, ^{
            
            [self updateImage];
            
            isDone = true;
            
        });
        
        
    } // endif isDone
    
    
    
    
}


@end
