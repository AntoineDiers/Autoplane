// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod backend;

use tauri::Manager;
use crate::backend::Backend;

fn main() {

    tauri::Builder::default()
        .plugin(tauri_plugin_opener::init())
        .invoke_handler(tauri::generate_handler![])
        .setup(|app|
        {
            let backend = Backend::new(app.handle().clone())?;
            app.manage(backend.clone());
            Ok(())
        })
        .run(tauri::generate_context!())
        .unwrap();
}
