// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod backend;
mod events;

use core::time;
use std::sync::{Arc, Mutex};
use std::thread::{self, sleep};
use tauri::Manager;

use crate::backend::Backend;


fn main() {

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![])
        .setup(|app|
        {
            let backend = Arc::new(Mutex::new(Backend::new(app.handle().clone())));
            app.manage(backend.clone());

            thread::spawn(move || 
            {
                loop
                {
                    backend.lock().unwrap().tick();
                    sleep(time::Duration::from_millis(100));
                }
            });

            Ok(())
        })
        .run(tauri::generate_context!())
        .unwrap();
}
