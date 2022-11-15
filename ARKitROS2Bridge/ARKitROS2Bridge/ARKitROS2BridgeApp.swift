//
//  AppDelegate.swift
//  ARKitROS2Bridge
//
//  Created by occamlab on 11/15/22.
//

import SwiftUI

@main
struct ARKitROS2BridgeApp: App {
    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(DataStreamer())
        }
    }
}
