//
//  ViewController.h
//  InfiniTAM
//
//  Created by Victor Adrian Prisacariu on 29/10/2014.
//  Copyright (c) 2014 Victor Adrian Prisacariu. All rights reserved.
//

#import <UIKit/UIKit.h>
#define HAS_LIBCXX
#import <Structure/Structure.h>
#import <CoreMotion/CoreMotion.h>

// true depth
#import <AVFoundation/AVFoundation.h>
#import <CoreVideo/CoreVideo.h>
#import <MobileCoreServices/MobileCoreServices.h>
#import <Accelerate/Accelerate.h>


@interface ViewController : UIViewController <STSensorControllerDelegate, AVCaptureDataOutputSynchronizerDelegate>

@property (weak, nonatomic) IBOutlet UIView *renderView;
@property (weak, nonatomic) IBOutlet UITextField *tbOut;
@property (weak, nonatomic) IBOutlet UIButton *bProcessOne;
@property (weak, nonatomic) IBOutlet UIButton *bProcessCont;

@property (nonatomic, strong) CMMotionManager *motionManager;

- (IBAction)bProcessOne_clicked:(id)sender;
- (IBAction)bProcessCont_clicked:(id)sender;

@end

