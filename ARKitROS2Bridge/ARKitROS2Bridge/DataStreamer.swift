//
//  DataStreamer.swift
//  ARKitROS2Bridge
//
//  Created by occamlab on 11/15/22.
//

import Foundation

class DataStreamer: ObservableObject {
    @Published var ipAddressText: String = ""
    
    var broadcastPoseConnection: UDPBroadcastConnection!
    var broadcastImagesConnection: UDPBroadcastConnection!
    
    var poseTimer = Timer()
    var imageTimer = Timer()
    
    var isProcessingFrame = false
    var imageIndex = 0                 // this is the sequence number in the image stream
    
    @Published var isSendingData: Bool = false
    
    @Published var arView = ARViewController()
    
    /// Initializes UDP connections for sending data to ROS
    func setupUdpConnections() {
        let INADDR_BROADCAST = in_addr(s_addr: inet_addr(ipAddressText))
        
        broadcastPoseConnection = UDPBroadcastConnection(port: Config.Ports.broadcast, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
        
        broadcastImagesConnection = UDPBroadcastConnection(port: Config.Ports.broadcastImages, ip: INADDR_BROADCAST) {(port: Int, response: [UInt8]) -> Void in
            print("Received from \(INADDR_BROADCAST):\(port):\n\n\(response)")
        }
    }
    
    /// Sends selected types of data to ROS when button is pressed
    func startButtonTapped() {
        if !isSendingData {
            print("Data stream started to IP \(ipAddressText)")
            poseTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitPoseData), userInfo: nil, repeats: true)
            imageTimer = Timer.scheduledTimer(timeInterval: 0.1, target: self, selector: #selector(self.transmitImages), userInfo: nil, repeats: true)
            isSendingData = true
        }
        else {
            poseTimer.invalidate()
            poseTimer = Timer()

            imageTimer.invalidate()
            imageTimer = Timer()
            isSendingData = false
        }
    }
    
    /// Sends the pose data to ROS
    @objc func transmitPoseData() {
        broadcastPoseConnection.sendBroadcast(arView.getCameraCoordinates())
    }
    
    /// Sends the camera frames to ROS
    @objc func transmitImages() {
        let intrinsics = arView.getCameraIntrinsics()
        let MTU = 1350
        let (image, stampedTime) = arView.getVideoFrames()
        let imageData = image.jpegData(compressionQuality: 0)
        let frameTime = String(stampedTime).data(using: .utf8)!
        let timeAndIntrinsics = frameTime + intrinsics
        var bytesSent = 0           // Keeps track of how much of the image has been sent
        var packetIndex = 0         // Packet number - so ROS can recompile the image in order
        
        while bytesSent < imageData!.count {
            // Construct the range for the packet
            let range = (bytesSent..<min(bytesSent + MTU, imageData!.count))
            var udpPacketPayload = imageData!.subdata(in: range)
            udpPacketPayload.insert(UInt8(imageIndex % (Int(UInt8.max) + 1)), at: 0)
            udpPacketPayload.insert(UInt8(packetIndex), at: 1)
            
            if bytesSent == 0 {
                let numPackets = (Float(imageData!.count) / Float(MTU)).rounded(.up)
                udpPacketPayload.insert(UInt8(numPackets), at: 2)
                udpPacketPayload.insert(UInt8(frameTime.count), at: 3)
                udpPacketPayload.insert(UInt8(intrinsics.count), at: 4)
                udpPacketPayload.insert(contentsOf: timeAndIntrinsics, at: 5)
            }
            broadcastImagesConnection.sendBroadcast(udpPacketPayload)
            bytesSent += range.count
            packetIndex += 1
        }
        imageIndex += 1
    }
}
