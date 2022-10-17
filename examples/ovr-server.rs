extern crate ovr_sys;
extern crate nalgebra as na;
extern crate pad_motion;

use ovr_sys::*;
// use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
// use na::{UnitQuaternion,Quaternion};

use pad_motion::protocol::*;
use pad_motion::server::*;

// fn scale(old_min: f32, old_max: f32, new_min: f32, new_max: f32, old_value: f32) -> f32{
 
//     let old_range: f32 = old_max - old_min;
//     let new_range: f32 = new_max - new_min;
//     let new_value: f32 = (((old_value - old_min) * new_range) / old_range) + new_min;
 
//     return new_value;
// }

fn my_try<F>(f: F) -> Result<(), Box<ovrErrorInfo>> where F: FnOnce() -> ovrResult {
    let result = f();
    if OVR_SUCCESS(result) {
        Ok(())
    } else {
        let mut info = Box::new(unsafe{::std::mem::zeroed()});
        unsafe{ ovr_GetLastErrorInfo(&mut *info as *mut _) }
        Err(info)
    }
}

fn main() {
    let running = Arc::new(AtomicBool::new(true));

    {
        let running = running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
        })
        .expect("Error setting Ctrl-C handler");
    }

    let server = Arc::new(Server::new(None, None).unwrap());
    let server_thread_join_handle = {
        let server = server.clone();
        server.start(running.clone())
    };

    let controller_info = ControllerInfo {
        slot_state: SlotState::Connected,
        device_type: DeviceType::FullGyro,
        connection_type: ConnectionType::USB,
        ..Default::default()
    };
    server.update_controller_info(controller_info);

    unsafe {
        let mut params: ovrInitParams = ::std::mem::zeroed();
        params.Flags |= ovrInit_RequestVersion;
        params.RequestedMinorVersion = OVR_MINOR_VERSION;
        my_try(|| ovr_Initialize(&params as *const _)).unwrap();
        let mut session: ovrSession = ::std::mem::zeroed();
        let mut luid: ovrGraphicsLuid = ::std::mem::zeroed();
        my_try(|| ovr_Create(&mut session as *mut _, &mut luid as *mut _)).unwrap();
        assert!(!session.is_null());
        println!("{:?}", luid);

        // let mut virt_e = (0.0_f32,0.0_f32,0.0_f32);
        // let mut delta_e = (0.0_f32,0.0_f32,0.0_f32);
        // let mut prev_e = (0.0_f32,0.0_f32,0.0_f32);

        // let G = 9.8;

        let now = Instant::now();
        while running.load(Ordering::SeqCst) {
            let time = ovr_GetPredictedDisplayTime(
                session,
                0);
            let t_state: ovrTrackingState = ovr_GetTrackingState(
                session,
                time,
                0
            );

            // let head_orientation = t_state.HeadPose.ThePose.Orientation;
            // let calib_orig = t_state.CalibratedOrigin.Position;

            // let _lin_vel = t_state.HeadPose.LinearVelocity; // m/s
            // let mut lin_vel = (_lin_vel.x, _lin_vel.y, _lin_vel.z);

            let _lin_acc = t_state.HeadPose.LinearAcceleration;  // m/s^2
            let lin_acc = (_lin_acc.x, _lin_acc.y, _lin_acc.z);

            let _ang_vel = t_state.HeadPose.AngularVelocity; // r/s
            let mut ang_vel = (_ang_vel.x, _ang_vel.y, _ang_vel.z);

            // let _ang_acc = t_state.HeadPose.AngularAcceleration; // rad/s^2
            // let mut ang_acc = (_ang_acc.x, _ang_acc.y, _ang_acc.z);

            ang_vel.0 = ang_vel.0 * 180.0 / std::f32::consts::PI;
            ang_vel.1 = ang_vel.1 * 180.0 / std::f32::consts::PI;
            ang_vel.2 = ang_vel.2 * 180.0 / std::f32::consts::PI;

            ang_vel.1 *= -1.0;
            ang_vel.2 *= -1.0;


            let controller_data = ControllerData {
                connected: true,
                motion_data_timestamp: now.elapsed().as_micros() as u64,

                accelerometer_x: lin_acc.2,
                accelerometer_y: lin_acc.1,
                accelerometer_z: lin_acc.0,
                
                gyroscope_roll:  ang_vel.2,
                gyroscope_yaw:   ang_vel.1,
                gyroscope_pitch: ang_vel.0,
                
                ..Default::default()
            };

            server.update_controller_data(0, controller_data);

            std::thread::sleep(Duration::from_millis(10));
        }

        server_thread_join_handle.join().unwrap();

        ovr_Destroy(session);
        ovr_Shutdown();
        // io::stdin().read_line(&mut String::new()).unwrap();
    }


}
