//
//  ViewController.swift
//  ARKit ROS2 Bridge
//  With modifications, ViewController is from: https://github.com/occamLab/ARKit-Ros-Bridge
//
//  Created by al-li-son on 11/14/22.
//

import UIKit
import ARKit

class ARViewController: UIViewController, ARSessionDelegate, ObservableObject {
    
    let configuration = ARWorldTrackingConfiguration()
    
    // Create an AR view
    var arView: ARSCNView! {
        get {
            return self.view as? ARSCNView
        }
        set(newView) {
            self.view = newView
        }
    }
    
    override func loadView() {
      self.view = ARSCNView(frame: .zero)
    }
    
    // Load, assign a delegate, and create a scene
    override func viewDidLoad() {
        super.viewDidLoad()
        arView.session.delegate = self
        arView.scene = SCNScene()
    }
    
    // Functions for standard AR view handling
    override func viewDidAppear(_ animated: Bool) {
       super.viewDidAppear(animated)
    }
    override func viewDidLayoutSubviews() {
       super.viewDidLayoutSubviews()
    }
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        arView.session.run(configuration)
    }
    override func viewWillDisappear(_ animated: Bool) {
       super.viewWillDisappear(animated)
       arView.session.pause()
    }
    
    /// Get video frames.
    func getVideoFrames() -> (UIImage, Double) {
        let cameraFrame = arView.session.currentFrame
        let stampedTime = cameraFrame?.timestamp
        
        // Convert ARFrame to a UIImage
        let pixelBuffer = cameraFrame?.capturedImage
        let ciImage = CIImage(cvPixelBuffer: pixelBuffer!)
        let context = CIContext(options: nil)
        let cgImage = context.createCGImage(ciImage, from: ciImage.extent)
        let uiImage = UIImage(cgImage: cgImage!)
        return (uiImage, stampedTime!)
    }
    
    /// Get pose data (transformation matrix, time) and send to ROS.
    func getCameraCoordinates() -> String {
        let camera = arView.session.currentFrame?.camera
        let cameraTransform = camera?.transform
        let relativeTime = arView.session.currentFrame?.timestamp
        let scene = SCNMatrix4(cameraTransform!)
        
        let fullMatrix = String(format: "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", scene.m11, scene.m12, scene.m13, scene.m14, scene.m21, scene.m22, scene.m23, scene.m24, scene.m31, scene.m32, scene.m33, scene.m34, scene.m41, scene.m42, scene.m43, scene.m44, relativeTime!)
        
        return fullMatrix
    }
        
    /// Get the camera intrinsics to send to ROS
    func getCameraIntrinsics(_ resizeFactor: Float) -> Data {
        let camera = arView.session.currentFrame?.camera
        let intrinsics = camera?.intrinsics
        let columns = intrinsics?.columns
        let res = camera?.imageResolution
        let width = res?.width
        let height = res?.height
        
        return String(format: "%f,%f,%f,%f,%f,%f,%f", columns!.0.x*resizeFactor, columns!.1.y*resizeFactor, columns!.2.x*resizeFactor, columns!.2.y*resizeFactor, columns!.2.z*resizeFactor, width!*CGFloat(resizeFactor), height!*CGFloat(resizeFactor)).data(using: .utf8)!
        }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }
    
}
