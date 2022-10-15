# rust-ovr-cemuhook
attempting to route OVR HMD tracking (orientation) data to Yuzu/Cemu via the Cemuhook protocol

this is a combination of [zduny's pad-motion](https://github.com/zduny/pad_motion/) rust implementation, 

combined with the [dylanede's ovr-sys](https://github.com/dylanede/ovr-sys) libOVR bindings crate

1. the idea is to gather ovrTrackingState.HeadPose.ThePose.Orientation every 10ms or so,

2. mix it with ovrTrackingState.CalibratedOrigin, 

3. then pass the deltas to [yuzu](https://github.com/yuzu-emu/yuzu) using the `cemuhook` protocol over a little UDP server. 

tested using PadTest.exe from here: [https://cemuhook.sshnuke.net/padudpserver.html](https://cemuhook.sshnuke.net/padudpserver.html)

seems to be working with super mario odyssey 1.3.0 in yuzu. still need to test breath of the wild.

it's a little janky, but perhaps better than taping an android phone to your HMD :shrug:

### 10/15/22
nearly working, tho, the values are not right. lots of gimbal locking happening. nearly playable. but mostly a mess

![image](https://user-images.githubusercontent.com/1683122/195977309-719c7902-a4fd-41d3-9f5b-25dbbb8bc77e.png)

![image](https://user-images.githubusercontent.com/1683122/195977320-03bff96a-7705-46b5-9a3d-f5a1b1e1ab1e.png)
