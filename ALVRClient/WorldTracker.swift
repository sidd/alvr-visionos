//
//  WorldTracker.swift
//  ALVRClient
//
//

import Foundation
import ARKit
import RealityKit
import CompositorServices

class WorldTracker {
    static let shared = WorldTracker()

    let arSession: ARKitSession!
    let worldTracking: WorldTrackingProvider!
    let handTracking: HandTrackingProvider!

    var deviceAnchorsLock = NSObject()
    var deviceAnchorsQueue = [UInt64]()
    var deviceAnchorsDictionary = [UInt64: simd_float4x4]()
    var leftHand: HandAnchor! = nil
    var rightHand: HandAnchor! = nil

    static let maxPrediction = 30 * NSEC_PER_MSEC
    static let deviceIdHead = alvr_path_string_to_id("/user/head")
    static let deviceIdLeftHand = alvr_path_string_to_id("/user/hand/left")
    static let deviceIdRightHand = alvr_path_string_to_id("/user/hand/right")

    let originEntity = AnchorEntity(world: .zero)
    let leftEntity = AnchorEntity()
    let rightEntity = AnchorEntity()

    init(arSession: ARKitSession = ARKitSession(), worldTracking: WorldTrackingProvider = WorldTrackingProvider(), handTracking: HandTrackingProvider = HandTrackingProvider()) {
        self.arSession = arSession
        self.worldTracking = worldTracking
        self.handTracking = handTracking
    }

    func initializeAr() async {
        do {
            initializeHandTracking()
            try await arSession.run([worldTracking, handTracking])
        } catch {
            fatalError("Failed to initialize ARSession")
        }
    }
    
    func initializeHandTracking() {
        Task {
            await leftEntity.setParent(originEntity)
            await rightEntity.setParent(originEntity)
            
            for await update in handTracking.anchorUpdates {
                switch update.event {
                case .updated:
                    let anchor = update.anchor
                    
                    guard anchor.isTracked else { continue }
                    
                    updateJoint(for: anchor)
                default:
                    break
                }
            }
        }
    }
    func updateJoint(for anchor: HandAnchor) {
        if anchor.chirality == .left {
            leftHand = anchor
        } else {
            rightHand = anchor
        }
        for joint in anchor.handSkeleton!.allJoints {
            let name = "\(anchor.chirality.description):\(joint.name)"
            if let entity = originEntity.findEntity(named: name) {
                if joint.isTracked {
                    entity.setTransformMatrix(anchor.originFromAnchorTransform * joint.anchorFromJointTransform, relativeTo: nil)
                    entity.scale = simd_float3(repeating: 0.01)
                    entity.isEnabled = true
                } else {
                    entity.isEnabled = false
                }
            } else {
                guard joint.isTracked else { continue }
                let entity = ModelEntity()
                entity.name = name
                entity.setTransformMatrix(anchor.originFromAnchorTransform * joint.anchorFromJointTransform, relativeTo: nil)
                entity.scale = simd_float3(repeating: 0.01)
                if anchor.chirality == .left {
                    leftEntity.addChild(entity)
                } else {
                    rightEntity.addChild(entity)
                }
            }
        }
    }

    // TODO: figure out how stable Apple's predictions are into the future
    
    func sendTracking(targetTimestamp: Double) {
        var targetTimestampWalkedBack = targetTimestamp
        var deviceAnchor:DeviceAnchor? = nil
        
        // Predict as far into the future as Apple will allow us.
        for i in 0...20 {
            deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: targetTimestampWalkedBack)
            if deviceAnchor != nil {
                break
            }
            targetTimestampWalkedBack -= (5/1000.0)
        }
        
        // Fallback.
        if deviceAnchor == nil {
            targetTimestampWalkedBack = CACurrentMediaTime()
            deviceAnchor = worldTracking.queryDeviceAnchor(atTimestamp: targetTimestamp)
        }

        // Well, I'm out of ideas.
        guard let deviceAnchor = deviceAnchor else {
            return
        }
        
        let targetTimestampNS = UInt64(targetTimestampWalkedBack * Double(NSEC_PER_SEC))
        
        deviceAnchorsQueue.append(targetTimestampNS)
        if deviceAnchorsQueue.count > 1000 {
            let val = deviceAnchorsQueue.removeFirst()
            deviceAnchorsDictionary.removeValue(forKey: val)
        }
        deviceAnchorsDictionary[targetTimestampNS] = deviceAnchor.originFromAnchorTransform
        let orientation = simd_quaternion(deviceAnchor.originFromAnchorTransform)
        let position = deviceAnchor.originFromAnchorTransform.columns.3
        let pose = AlvrPose(orientation: AlvrQuat(x: orientation.vector.x, y: orientation.vector.y, z: orientation.vector.z, w: orientation.vector.w), position: (position.x, position.y, position.z))
        var headTrackingMotion = AlvrDeviceMotion(device_id: WorldTracker.deviceIdHead, pose: pose, linear_velocity: (0, 0, 0), angular_velocity: (0, 0, 0))

        var leftHandTrackingMotion: AlvrDeviceMotion = AlvrDeviceMotion()
        if let leftHand {
            let leftOrientation = simd_quaternion(leftHand.originFromAnchorTransform)
            let leftPosition = leftHand.originFromAnchorTransform.columns.3
            let leftHandPose = AlvrPose(orientation: AlvrQuat(x: leftOrientation.vector.x, y: leftOrientation.vector.y, z: leftOrientation.vector.z, w: leftOrientation.vector.w), position: (leftPosition.x, leftPosition.y, leftPosition.z))
            leftHandTrackingMotion = AlvrDeviceMotion(device_id: WorldTracker.deviceIdLeftHand, pose: leftHandPose, linear_velocity: (0, 0, 0), angular_velocity: (0, 0, 0))
        }


        var rightHandTrackingMotion: AlvrDeviceMotion = AlvrDeviceMotion()
        if let rightHand {
            let rightOrientation = simd_quaternion(rightHand.originFromAnchorTransform)
            let rightPosition = rightHand.originFromAnchorTransform.columns.3
            let rightHandPose = AlvrPose(orientation: AlvrQuat(x: rightOrientation.vector.x, y: rightOrientation.vector.y, z: rightOrientation.vector.z, w: rightOrientation.vector.w), position: (rightPosition.x, rightPosition.y, rightPosition.z))
            rightHandTrackingMotion = AlvrDeviceMotion(device_id: WorldTracker.deviceIdRightHand, pose: rightHandPose, linear_velocity: (0, 0, 0), angular_velocity: (0, 0, 0))
        }

        let targetTimestampReqestedNS = UInt64(targetTimestamp * Double(NSEC_PER_SEC))
        let currentTimeNs = UInt64(CACurrentMediaTime() * Double(NSEC_PER_SEC))
        //print("asking for:", targetTimestampNS, "diff:", targetTimestampReqestedNS&-targetTimestampNS, "diff2:", targetTimestampNS&-lastRequestedTimestamp, "diff3:", targetTimestampNS&-currentTimeNs)
        EventHandler.shared.lastRequestedTimestamp = targetTimestampNS
        let posesArr = [getAlvrPoses(for: .left), getAlvrPoses(for: .right)]

        var posesPtr: UnsafePointer<UnsafePointer<AlvrPose>?>? = nil
        let posesMutPtr: UnsafeMutablePointer<UnsafePointer<AlvrPose>?> = UnsafeMutablePointer<UnsafePointer<AlvrPose>?>.allocate(capacity: posesArr.count)

        for (index, poses) in posesArr.enumerated() {
            let innerArrayPointer = UnsafeMutablePointer<AlvrPose>.allocate(capacity: poses.count)

            poses.withUnsafeBufferPointer { buffer in
                innerArrayPointer.initialize(from: buffer.baseAddress!, count: buffer.count)
            }

            posesMutPtr[index] = UnsafePointer(innerArrayPointer)
            posesPtr = UnsafePointer(posesMutPtr)
        }

        var trackingMotions = [headTrackingMotion, leftHandTrackingMotion, rightHandTrackingMotion]

        alvr_send_tracking(targetTimestampNS, &trackingMotions, 3, posesPtr, nil)
   }

    func getAlvrPoses(for chirality: HandAnchor.Chirality) -> [AlvrPose] {
        let entity = chirality == .left ? leftEntity : rightEntity

        var poses: [AlvrPose] = [AlvrPose()]
        for child in entity.children {
            let position = child.position(relativeTo: nil)
            let orientation = child.orientation(relativeTo: nil).vector
            let quat = AlvrQuat(x: orientation.x, y: orientation.y, z: orientation.z, w: orientation.w)
            let pose = AlvrPose(orientation: quat, position: (position.x, position.y, position.z))
            poses.append(pose)
        }
        
        return poses
    }
    
    func lookupDeviceAnchorFor(timestamp: UInt64) -> simd_float4x4? {
        return deviceAnchorsDictionary[timestamp]
    }
    
}
