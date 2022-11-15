//
//  ContentView.swift
//  ARKitROS2Bridge
//
//  Created by occamlab on 11/15/22.
//

import SwiftUI

struct ContentView : View {
    @State var ipAddress: String = ""
    @EnvironmentObject var dataStreamer: DataStreamer
    
    var body: some View {
        VStack {
            HStack {
                TextField(
                    "IP Address",
                    text: $ipAddress
                )
                Button {
                    dataStreamer.ipAddressText = ipAddress
                    dataStreamer.setupUdpConnections()
                } label: {
                    Text("Submit")
                }.padding()
            }
            NavigationIndicator()
                .environmentObject(dataStreamer)
                .edgesIgnoringSafeArea(.all)
            
            Button {
                dataStreamer.startButtonTapped()
            } label: {
                if dataStreamer.isSendingData {
                    Text("Stop Data Stream")
                        .foregroundColor(.red)
                } else {
                    Text("Start Data Stream")
                }
            }.disabled(dataStreamer.ipAddressText=="")
        }
    }
}

struct NavigationIndicator: UIViewControllerRepresentable {
    @EnvironmentObject var dataStreamer: DataStreamer
    typealias UIViewControllerType = ARViewController
    func makeUIViewController(context: Context) -> ARViewController {
        return dataStreamer.arView
    }
    func updateUIViewController(_ uiViewController:
    NavigationIndicator.UIViewControllerType, context:
    UIViewControllerRepresentableContext<NavigationIndicator>) {
       
    }
}

#if DEBUG
struct ContentView_Previews : PreviewProvider {
    static var previews: some View {
        ContentView()
    }
}
#endif
