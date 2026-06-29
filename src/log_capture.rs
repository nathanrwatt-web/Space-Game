// For the command line log in game 
// Should essentially mirror the log in bevy 
// For now completely AI generated, needs to be reviewed in depth

use bevy::prelude::*;
use bevy::log::{BoxedLayer, Level};
use bevy::log::tracing::{Event, Subscriber};
use bevy::log::tracing::field::{Field, Visit};
use bevy::log::tracing_subscriber::Layer;
use bevy::log::tracing_subscriber::layer::Context;
use bevy_egui::{egui, EguiContexts};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};

const MAX_LINES: usize = 300;

pub struct LogLine {
    pub level: Level,
    pub msg: String,
}

#[derive(Resource)]
pub struct LogStore(pub Arc<Mutex<VecDeque<LogLine>>>);

struct CaptureLayer {
    store: Arc<Mutex<VecDeque<LogLine>>>,
}

struct MsgVisitor(Option<String>);
impl Visit for MsgVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn std::fmt::Debug) {
        if field.name() == "message" {
            self.0 = Some(format!("{value:?}"));
        }
    }
}
#[allow(clippy::collapsible_if)] // only part of the if is collapsable, looks better like this 
impl<S: Subscriber> Layer<S> for CaptureLayer {
    fn on_event(&self, event: &Event<'_>, _ctx: Context<'_, S>) {
        let mut vis = MsgVisitor(None);
        event.record(&mut vis);
        if let Some(msg) = vis.0 {
            if let Ok(mut buf) = self.store.lock() {
                if buf.len() >= MAX_LINES {
                    buf.pop_front();
                }
                buf.push_back(LogLine { level: *event.metadata().level(), msg });
            }
        }
    }
}

// LogPlugin.custom_layer is a plain fn pointer; it can still touch the App to share the buffer.
pub fn capture_layer(app: &mut App) -> Option<BoxedLayer> {
    let store = Arc::new(Mutex::new(VecDeque::new()));
    app.insert_resource(LogStore(store.clone()));
    Some(Box::new(CaptureLayer { store }))
}

// ===== the window =====
#[derive(Resource, Default)] // default implies off on first start 
pub struct LogWindow {
    pub open: bool,
}


pub fn log_panel(
    mut contexts: EguiContexts,
    store: Res<LogStore>,
    mut win: ResMut<LogWindow>,
) -> Result {
    if !win.open {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    let mut open = win.open;
    egui::Window::new("Log")
        .open(&mut open)
        .default_width(440.0)
        .default_height(220.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
                if let Ok(buf) = store.0.lock() {
                    for line in buf.iter() {
                        ui.colored_label(color_for(line.level), line.msg.as_str());
                    }
                }
            });
        });
    win.open = open;
    Ok(())
}

fn color_for(level: Level) -> egui::Color32 {
    if level == Level::ERROR {
        egui::Color32::from_rgb(240, 90, 90)
    } else if level == Level::WARN {
        egui::Color32::from_rgb(240, 200, 90)
    } else if level == Level::INFO {
        egui::Color32::from_rgb(180, 220, 180)
    } else {
        egui::Color32::GRAY
    }
}
