extern crate ovr_sys;
extern crate nalgebra as na;
extern crate pad_motion;

use ovr_sys::*;
// use std::io;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};
use na::{UnitQuaternion,Quaternion};

use pad_motion::protocol::*;
use pad_motion::server::*;

fn scale(old_min: f32, old_max: f32, new_min: f32, new_max: f32, old_value: f32) -> f32{
 
    let old_range: f32 = old_max - old_min;
    let new_range: f32 = new_max - new_min;
    let new_value: f32 = (((old_value - old_min) * new_range) / old_range) + new_min;
 
    return new_value;
}

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

        let mut virt_e = (0.0_f32,0.0_f32,0.0_f32);
        let mut delta_e = (0.0_f32,0.0_f32,0.0_f32);
        let mut prev_e = (0.0_f32,0.0_f32,0.0_f32);

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

            let head_orientation = t_state.HeadPose.ThePose.Orientation;
            let calib_orig = t_state.CalibratedOrigin.Position;

            // println!("{:?},{:?},{:?},{:?}", 
            //     head_orientation.x,
            //     head_orientation.y,
            //     head_orientation.z,
            //     head_orientation.w);

            let quat = UnitQuaternion::from_quaternion(Quaternion::new(
                head_orientation.x,
                head_orientation.y,
                head_orientation.z,
                head_orientation.w,
            ));

            let calib_quat = UnitQuaternion::from_quaternion(Quaternion::new(
                t_state.CalibratedOrigin.Orientation.x,
                t_state.CalibratedOrigin.Orientation.y,
                t_state.CalibratedOrigin.Orientation.z,
                t_state.CalibratedOrigin.Orientation.w
            ));

            let mut rot_dif = quat.rotation_to(&calib_quat).euler_angles();


            // let inv = quat.inverse();

            //https://github.com/dimforge/nalgebra/blob/1e9e1ba46d8d609901932063e31022b2f2e3d277/src/geometry/rotation_specialization.rs
            let mut angs = quat.euler_angles(); // (roll,yaw,pitch)

            // angs.0 = scale(-3.0, 3.0, -180.0, 180.0, angs.0);
            // angs.1 = scale(-1.5, 1.5, -90.0, 90.0, angs.1);
            // angs.2 = scale(-3.0, 3.0, -90.0, 90.0, angs.2);

            // rot_dif.0 = f32::trunc(rot_dif.0  * 100.0) / 100.0;
            // rot_dif.1 = f32::trunc(rot_dif.1  * 100.0) / 100.0;
            // rot_dif.2 = f32::trunc(rot_dif.2  * 100.0) / 100.0;

            rot_dif.0 = scale(-1.0, 1.0, -90.0, 90.0, rot_dif.0);
            rot_dif.1 = scale(-1.0, 1.0, -90.0, 90.0, rot_dif.1);
            rot_dif.2 = scale(-1.0, 1.0, -90.0, 90.0, rot_dif.2);

            // angs.0 = angs.0 * 180.0 / std::f32::consts::PI;
            // angs.1 = angs.1 * 180.0 / std::f32::consts::PI;
            // angs.2 = angs.2 * 180.0 / std::f32::consts::PI;

            rot_dif.0 *= 90.0; // roll
            rot_dif.0 *= -1.0;

            rot_dif.1 *= 90.0; // yaw
            rot_dif.1 *= -1.0;
            
            rot_dif.2 *= 80.0; // pitch
            // rot_dif.2 *= -1.0;

            delta_e.0 = rot_dif.0 - prev_e.0;
            delta_e.1 = rot_dif.1 - prev_e.1;
            delta_e.2 = rot_dif.2 - prev_e.2;

            println!("{:.2}\t{:.2}\t{:.2} | \t{:.2}\t{:.2}\t{:.2}",
                rot_dif.0,
                rot_dif.1,
                rot_dif.2,
                delta_e.0,
                delta_e.1,
                delta_e.2);

            

            // println!("0:{:.1}\t1:{:.1}\t2:{:.1}\t | {:.1}\t{:.1}\t{:.1}", 
            //     angs.0,
            //     angs.1,
            //     angs.2,
            //     delta_e.0,
            //     delta_e.1,
            //     delta_e.2);


            let controller_data = ControllerData {
                connected: true,
                motion_data_timestamp: now.elapsed().as_micros() as u64,
                gyroscope_roll: delta_e.1,
                gyroscope_yaw: delta_e.0,
                gyroscope_pitch: delta_e.2,
                ..Default::default()
            };

            server.update_controller_data(0, controller_data);

            virt_e.0 += delta_e.0;
            virt_e.1 += delta_e.1;
            virt_e.2 += delta_e.2;

            prev_e = rot_dif.clone();

            std::thread::sleep(Duration::from_millis(10));
        }

        server_thread_join_handle.join().unwrap();

        ovr_Destroy(session);
        ovr_Shutdown();
        // io::stdin().read_line(&mut String::new()).unwrap();
    }


}
