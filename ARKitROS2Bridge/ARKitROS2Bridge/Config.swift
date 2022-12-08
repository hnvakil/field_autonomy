//
//  Config.swift
//  ARKit ROS2 Bridge
//  With minor modifications, Config.swift is from: https://github.com/occamLab/ARKit-Ros-Bridge
//
//  Created by al-li-son on 11/14/22.
//

import Foundation

struct Config {
    struct Ports {
        static let broadcast = UInt16(35601)
        static let broadcastImages = UInt16(35602)
        static let broadcastGPS = UInt16(35603)
    }
    
}
