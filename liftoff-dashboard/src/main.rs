use clap::Parser;
use crossterm::event::{self, Event, KeyCode, KeyEventKind, KeyModifiers};
use liftoff_lib::crsf::{self, CrsfPacket};
use liftoff_lib::simstate::{self, BatteryPacket};
use liftoff_lib::topics;
use log::{info, warn};
use ratatui::layout::{Alignment, Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols;
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Axis, Block, Borders, Chart, Dataset, List, ListItem, ListState, Paragraph,
};
use ratatui::{DefaultTerminal, Frame};
use std::collections::VecDeque;
use std::sync::{Arc, RwLock};
use std::time::Duration;
use zenoh::Config;

// ---------------------------------------------------------------------------
// CLI
// ---------------------------------------------------------------------------

#[derive(Parser, Debug)]
#[command(author, version, about = "Real-time telemetry dashboard for Liftoff")]
struct Args {
    /// Zenoh connect endpoint (e.g. tcp/192.168.1.1:7447). Omit for peer discovery.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh mode (peer or client).
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Zenoh topic prefix.
    #[arg(long, default_value = topics::DEFAULT_PREFIX)]
    zenoh_prefix: String,
}

// ---------------------------------------------------------------------------
// Ring-buffered time series
// ---------------------------------------------------------------------------

const DEFAULT_CAPACITY: usize = 1200; // ~2 min at 10 Hz

struct Series {
    name: String,
    color: Color,
    /// Raw samples; None = missing.
    raw: VecDeque<Option<f64>>,
    capacity: usize,
}

impl Series {
    fn new(name: &str, color: Color) -> Self {
        Self {
            name: name.to_string(),
            color,
            raw: VecDeque::with_capacity(DEFAULT_CAPACITY),
            capacity: DEFAULT_CAPACITY,
        }
    }

    fn push(&mut self, val: Option<f64>) {
        if self.raw.len() >= self.capacity {
            self.raw.pop_front();
        }
        self.raw.push_back(val);
    }

    fn last_value(&self) -> Option<f64> {
        self.raw.back().copied().flatten()
    }
}

// ---------------------------------------------------------------------------
// Graph panel (one Chart widget with N series)
// ---------------------------------------------------------------------------

struct Graph {
    label: &'static str,
    y_label: &'static str,
    series: Vec<Series>,
    legend: ListState,
    /// How many samples to display on screen.
    window: usize,
    /// Fixed Y-axis bounds. When set, auto-scaling is skipped.
    fixed_bounds: Option<[f64; 2]>,
}

impl Graph {
    fn new(label: &'static str, y_label: &'static str, series: Vec<Series>) -> Self {
        Self {
            label,
            y_label,
            series,
            legend: ListState::default(),
            window: 600,
            fixed_bounds: None,
        }
    }

    fn with_fixed_bounds(mut self, bounds: [f64; 2]) -> Self {
        self.fixed_bounds = Some(bounds);
        self
    }

    fn zoom_in(&mut self) {
        self.window = (self.window as f64 * 0.8).max(20.0) as usize;
    }

    fn zoom_out(&mut self) {
        self.window = (self.window as f64 * 1.25).min(DEFAULT_CAPACITY as f64) as usize;
    }

    fn select_next(&mut self) {
        let n = self.series.len();
        self.legend.select(match self.legend.selected() {
            Some(i) => Some((i + 1) % n),
            None => Some(0),
        });
    }

    fn select_prev(&mut self) {
        let n = self.series.len();
        self.legend.select(match self.legend.selected() {
            Some(0) => Some(n - 1),
            Some(i) => Some(i - 1),
            None => Some(0),
        });
    }

    fn unselect(&mut self) {
        self.legend.select(None);
    }

    /// Build (x,y) plot data for a series, relative to the display window.
    fn plot_data(&self, series_idx: usize) -> Vec<(f64, f64)> {
        let s = &self.series[series_idx];
        let len = s.raw.len();
        let start = len.saturating_sub(self.window);
        let mut pts = Vec::new();
        for (i, val) in s.raw.iter().skip(start).enumerate() {
            if let Some(v) = val {
                pts.push((i as f64, *v));
            }
        }
        pts
    }

    fn bounds(&self) -> [f64; 2] {
        if let Some(b) = self.fixed_bounds {
            return b;
        }
        let selected = self.legend.selected();
        let mut min: Option<f64> = None;
        let mut max: Option<f64> = None;
        for (idx, s) in self.series.iter().enumerate() {
            if let Some(sel) = selected
                && idx != sel
            {
                continue;
            }
            let len = s.raw.len();
            let start = len.saturating_sub(self.window);
            for val in s.raw.iter().skip(start).flatten() {
                min = Some(min.map_or(*val, |m: f64| m.min(*val)));
                max = Some(max.map_or(*val, |m: f64| m.max(*val)));
            }
        }
        let lo = min.unwrap_or(0.0);
        let hi = max.unwrap_or(1.0);
        // Add 15% padding; ensure non-zero range
        let margin = ((hi - lo).abs() * 0.15).max(0.5);
        [(lo - margin).max(if lo >= 0.0 { 0.0 } else { lo - margin }), hi + margin]
    }
}

// ---------------------------------------------------------------------------
// Shared telemetry state written by Zenoh subscriber tasks
// ---------------------------------------------------------------------------

#[derive(Default, Clone)]
struct TelemetryState {
    /// CRSF GPS altitude (m).
    altitude: Option<f64>,
    /// CRSF vario vertical speed (m/s).
    vario: Option<f64>,
    /// CRSF battery voltage (V).
    voltage: Option<f64>,
    /// CRSF battery current (A).
    current: Option<f64>,
    /// CRSF battery remaining (%).
    battery_pct: Option<f64>,
    /// CRSF attitude (pitch, roll, yaw) in degrees.
    pitch: Option<f64>,
    roll: Option<f64>,
    yaw: Option<f64>,
    /// CRSF airspeed (km/h).
    airspeed: Option<f64>,
    /// CRSF ground speed (km/h).
    ground_speed: Option<f64>,
    /// CRSF RPM values.
    rpms: Option<Vec<u32>>,
    /// GPS satellites.
    sats: Option<u8>,
    /// Heading (deg).
    heading: Option<f64>,
    /// Simstate damage per rotor.
    damage: Option<[f32; 4]>,
    damage_killed: bool,
    damage_crashed: bool,
    damage_no_drone: bool,
    /// Simstate battery (detailed).
    sim_battery: Option<BatteryPacket>,
    /// Whether we've received anything at all.
    connected: bool,
}

// ---------------------------------------------------------------------------
// Dashboard
// ---------------------------------------------------------------------------

struct Dashboard {
    graphs: Vec<Graph>,
    current_graph: usize,
}

impl Dashboard {
    fn new() -> Self {
        let alt_graph = Graph::new("Altitude", "Meters", vec![
            Series::new("GPS Alt", Color::Cyan),
        ]);

        let vario_graph = Graph::new("Vertical Speed", "m/s", vec![
            Series::new("Vario", Color::Yellow),
        ]);

        let battery_graph = Graph::new("Battery", "V / A / %", vec![
            Series::new("Voltage (V)", Color::Green),
            Series::new("Current (A)", Color::Red),
            Series::new("Remaining (%)", Color::Magenta),
        ]);

        let attitude_graph = Graph::new("Attitude", "Degrees", vec![
            Series::new("Pitch", Color::Yellow),
            Series::new("Roll", Color::Green),
            Series::new("Yaw", Color::Cyan),
        ])
        .with_fixed_bounds([-180.0, 180.0]);

        let speed_graph = Graph::new("Speed", "km/h", vec![
            Series::new("Ground", Color::LightBlue),
            Series::new("Air", Color::LightRed),
        ]);

        Self {
            graphs: vec![alt_graph, vario_graph, battery_graph, attitude_graph, speed_graph],
            current_graph: 0,
        }
    }

    fn ingest(&mut self, state: &TelemetryState) {
        // Altitude
        self.graphs[0].series[0].push(state.altitude);

        // Vario
        self.graphs[1].series[0].push(state.vario);

        // Battery
        self.graphs[2].series[0].push(state.voltage);
        self.graphs[2].series[1].push(state.current);
        self.graphs[2].series[2].push(state.battery_pct);

        // Attitude (convert to degrees for readability)
        self.graphs[3].series[0].push(state.pitch);
        self.graphs[3].series[1].push(state.roll);
        self.graphs[3].series[2].push(state.yaw);

        // Speed
        self.graphs[4].series[0].push(state.ground_speed);
        self.graphs[4].series[1].push(state.airspeed);
    }

    fn tab_next(&mut self) {
        self.current_graph = (self.current_graph + 1) % self.graphs.len();
    }

    fn tab_prev(&mut self) {
        self.current_graph = if self.current_graph == 0 {
            self.graphs.len() - 1
        } else {
            self.current_graph - 1
        };
    }
}

// ---------------------------------------------------------------------------
// Damage mini-panel (reused from liftoff-damage-indicator concepts)
// ---------------------------------------------------------------------------

fn damage_color(health: f32) -> Color {
    let h = health.clamp(0.0, 1.0);
    if h >= 0.5 {
        let t = (h - 0.5) * 2.0;
        Color::Rgb(255, 255, (t * 255.0) as u8)
    } else {
        let t = h * 2.0;
        Color::Rgb(255, (t * 255.0) as u8, 0)
    }
}

/// Mini drone diagram for the sidebar.
///
/// Layout (22 chars wide, 13 lines tall):
///
///    ◎           ◎       <- rotors (LF, RF)
///   100%       100%      <- percentages
///      ╲       ╱         <- arms
///       ┌─────┐          <- body
///       │  ◉  │
///       └─────┘
///      ╱       ╲         <- arms
///   100%       100%      <- percentages
///    ◎           ◎       <- rotors (LB, RB)
///
const MINI_DRONE_WIDTH: u16 = 22;
const MINI_DRONE_HEIGHT: u16 = 13;

fn render_damage_mini(f: &mut Frame, area: Rect, state: &TelemetryState) {
    let block = Block::default().borders(Borders::ALL).title(" Damage ");
    let inner = block.inner(area);
    f.render_widget(block, area);

    if inner.height < 3 || inner.width < 10 {
        return;
    }

    match state.damage {
        None => {
            let p = Paragraph::new("No data")
                .style(Style::default().fg(Color::DarkGray))
                .alignment(Alignment::Center);
            f.render_widget(p, inner);
        }
        Some(dmg) => {
            let override_color = if state.damage_no_drone {
                Some(Color::DarkGray)
            } else if state.damage_killed || state.damage_crashed {
                Some(Color::Red)
            } else {
                None
            };

            let body_color = override_color.unwrap_or(Color::White);
            let rotor_color = |idx: usize| override_color.unwrap_or_else(|| damage_color(dmg[idx]));

            // Status line at top
            let status_text = if state.damage_no_drone {
                Span::styled("No drone", Style::default().fg(Color::DarkGray))
            } else if state.damage_killed {
                Span::styled("KILLED", Style::default().fg(Color::Red))
            } else if state.damage_crashed {
                Span::styled("CRASHED", Style::default().fg(Color::Red))
            } else {
                let avg = dmg.iter().sum::<f32>() / 4.0;
                Span::styled(
                    format!("{:.0}%", avg * 100.0),
                    Style::default().fg(damage_color(avg)),
                )
            };

            // If we have enough room for the diagram, draw it; otherwise fall
            // back to the compact two-line version.
            if inner.height > MINI_DRONE_HEIGHT && inner.width >= MINI_DRONE_WIDTH {
                let rows = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Length(1),               // status
                        Constraint::Length(MINI_DRONE_HEIGHT), // drone
                        Constraint::Min(0),                  // remaining space
                    ])
                    .split(inner);

                f.render_widget(
                    Paragraph::new(Line::from(status_text)).alignment(Alignment::Center),
                    rows[0],
                );

                render_mini_drone(f, rows[1], &dmg, rotor_color, body_color);
            } else {
                // Fallback: compact text layout
                let rows = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([Constraint::Length(1); 3])
                    .split(inner);

                f.render_widget(
                    Paragraph::new(Line::from(status_text)).alignment(Alignment::Center),
                    rows[0],
                );

                let labels = ["LF", "RF", "LB", "RB"];
                let front: Vec<Span> = vec![
                    Span::styled(
                        format!(" {} {:.0}%", labels[0], dmg[0] * 100.0),
                        Style::default().fg(rotor_color(0)),
                    ),
                    Span::raw("  "),
                    Span::styled(
                        format!("{} {:.0}% ", labels[1], dmg[1] * 100.0),
                        Style::default().fg(rotor_color(1)),
                    ),
                ];
                f.render_widget(
                    Paragraph::new(Line::from(front)).alignment(Alignment::Center),
                    rows[1],
                );

                if rows.len() > 2 {
                    let back: Vec<Span> = vec![
                        Span::styled(
                            format!(" {} {:.0}%", labels[2], dmg[2] * 100.0),
                            Style::default().fg(rotor_color(2)),
                        ),
                        Span::raw("  "),
                        Span::styled(
                            format!("{} {:.0}% ", labels[3], dmg[3] * 100.0),
                            Style::default().fg(rotor_color(3)),
                        ),
                    ];
                    f.render_widget(
                        Paragraph::new(Line::from(back)).alignment(Alignment::Center),
                        rows[2],
                    );
                }
            }
        }
    }
}

fn render_mini_drone(
    f: &mut Frame,
    area: Rect,
    dmg: &[f32; 4],
    rotor_color: impl Fn(usize) -> Color,
    body_color: Color,
) {
    // Center the mini drone in the given area.
    let dx = area.x + area.width.saturating_sub(MINI_DRONE_WIDTH) / 2;
    let dy = area.y + area.height.saturating_sub(MINI_DRONE_HEIGHT) / 2;

    let put = |f: &mut Frame, x: u16, y: u16, text: &str, style: Style| {
        if y >= area.y + area.height || x >= area.x + area.width {
            return;
        }
        let w = (text.len() as u16).min(area.x + area.width - x);
        f.render_widget(
            Paragraph::new(Line::from(Span::styled(text, style))),
            Rect::new(x, y, w, 1),
        );
    };

    let pct_str = |idx: usize| -> String {
        format!("{:.0}%", dmg[idx] * 100.0)
    };

    // Row 0: front rotors
    let rc0 = rotor_color(0);
    let rc1 = rotor_color(1);
    put(f, dx + 3, dy, "◎", Style::default().fg(rc0));
    put(f, dx + 17, dy, "◎", Style::default().fg(rc1));

    // Row 1: front percentages
    let s0 = pct_str(0);
    let s1 = pct_str(1);
    let pad0 = 4u16.saturating_sub(s0.len() as u16 / 2);
    let pad1 = 17u16.saturating_sub(s1.len() as u16 / 2);
    put(f, dx + pad0, dy + 1, &s0, Style::default().fg(rc0));
    put(f, dx + pad1, dy + 1, &s1, Style::default().fg(rc1));

    // Rows 2-3: front arms (colored by attached rotor)
    let arm0 = Style::default().fg(rc0);
    let arm1 = Style::default().fg(rc1);
    put(f, dx + 6,  dy + 2, "╲", arm0);
    put(f, dx + 14, dy + 2, "╱", arm1);
    put(f, dx + 7,  dy + 3, "╲", arm0);
    put(f, dx + 13, dy + 3, "╱", arm1);

    // Rows 4-8: body
    let body_style = Style::default().fg(body_color);
    put(f, dx + 7, dy + 4, "┌─────┐", body_style);
    put(f, dx + 7, dy + 5, "│     │", body_style);
    put(f, dx + 7, dy + 6, "│  ◉  │", body_style);
    put(f, dx + 7, dy + 7, "│     │", body_style);
    put(f, dx + 7, dy + 8, "└─────┘", body_style);

    // Rows 9-10: back arms (colored by attached rotor)
    let rc2 = rotor_color(2);
    let rc3 = rotor_color(3);
    let arm2 = Style::default().fg(rc2);
    let arm3 = Style::default().fg(rc3);
    put(f, dx + 7,  dy + 9,  "╱", arm2);
    put(f, dx + 13, dy + 9,  "╲", arm3);
    put(f, dx + 6,  dy + 10, "╱", arm2);
    put(f, dx + 14, dy + 10, "╲", arm3);

    // Row 11: back percentages
    let s2 = pct_str(2);
    let s3 = pct_str(3);
    let pad2 = 4u16.saturating_sub(s2.len() as u16 / 2);
    let pad3 = 17u16.saturating_sub(s3.len() as u16 / 2);
    put(f, dx + pad2, dy + 11, &s2, Style::default().fg(rc2));
    put(f, dx + pad3, dy + 11, &s3, Style::default().fg(rc3));

    // Row 12: back rotors
    put(f, dx + 3,  dy + 12, "◎", Style::default().fg(rc2));
    put(f, dx + 17, dy + 12, "◎", Style::default().fg(rc3));
}

// ---------------------------------------------------------------------------
// Status bar
// ---------------------------------------------------------------------------

fn render_status_bar(f: &mut Frame, area: Rect, state: &TelemetryState) {
    let mut spans = Vec::new();

    if !state.connected {
        spans.push(Span::styled(
            " Waiting for telemetry...",
            Style::default().fg(Color::DarkGray),
        ));
    } else {
        if let Some(sats) = state.sats {
            spans.push(Span::styled("SAT ", Style::default().fg(Color::DarkGray)));
            spans.push(Span::styled(
                format!("{}", sats),
                Style::default().fg(if sats >= 6 { Color::Green } else { Color::Red }),
            ));
            spans.push(Span::raw("  "));
        }
        if let Some(alt) = state.altitude {
            spans.push(Span::styled("ALT ", Style::default().fg(Color::DarkGray)));
            spans.push(Span::raw(format!("{:.1}m", alt)));
            spans.push(Span::raw("  "));
        }
        if let Some(v) = state.voltage {
            spans.push(Span::styled("BAT ", Style::default().fg(Color::DarkGray)));
            spans.push(Span::raw(format!("{:.1}V", v)));
            spans.push(Span::raw("  "));
        }
        if let Some(hdg) = state.heading {
            spans.push(Span::styled("HDG ", Style::default().fg(Color::DarkGray)));
            spans.push(Span::raw(format!("{:.0}°", hdg)));
        }
    }

    let para = Paragraph::new(Line::from(spans)).alignment(Alignment::Left);
    f.render_widget(para, area);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------

fn draw_graph(f: &mut Frame, area: Rect, graph: &mut Graph, active: bool) {
    // Split: chart on left, legend on right.
    let legend_width = 28u16.min(area.width * 40 / 100);
    let cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Min(1),
            Constraint::Length(legend_width),
        ])
        .split(area);

    let bounds = graph.bounds();
    let window = graph.window;
    let selected = graph.legend.selected();

    // Build datasets
    let plot_data: Vec<Vec<(f64, f64)>> = (0..graph.series.len())
        .map(|i| graph.plot_data(i))
        .collect();

    let datasets: Vec<Dataset> = graph
        .series
        .iter()
        .enumerate()
        .filter(|(i, _)| selected.is_none() || selected == Some(*i))
        .map(|(i, s)| {
            Dataset::default()
                .name(s.name.as_str())
                .marker(symbols::Marker::Braille)
                .style(Style::default().fg(s.color))
                .data(&plot_data[i])
        })
        .collect();

    let border_style = if active {
        Style::default().fg(Color::Cyan)
    } else {
        Style::default().fg(Color::DarkGray)
    };

    let chart = Chart::new(datasets)
        .block(
            Block::default()
                .title(Span::styled(
                    graph.label,
                    Style::default()
                        .fg(Color::Cyan)
                        .add_modifier(Modifier::BOLD),
                ))
                .borders(Borders::ALL)
                .border_style(border_style),
        )
        .x_axis(
            Axis::default()
                .title("Time")
                .style(Style::default().fg(Color::Gray))
                .labels(vec![
                    Span::styled(
                        format!("t-{}", window),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(
                        "now".to_string(),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                ])
                .bounds([0.0, window as f64]),
        )
        .y_axis(
            Axis::default()
                .title(graph.y_label)
                .style(Style::default().fg(Color::Gray))
                .labels(vec![
                    Span::styled(
                        format!("{:.1}", bounds[0]),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                    Span::styled(
                        format!("{:.1}", bounds[1]),
                        Style::default().add_modifier(Modifier::BOLD),
                    ),
                ])
                .bounds(bounds),
        );

    f.render_widget(chart, cols[0]);

    // Legend panel
    let rows: Vec<ListItem> = graph
        .series
        .iter()
        .map(|s| {
            let val = match s.last_value() {
                Some(v) => format!("{:>8.2}", v),
                None => "       -".to_string(),
            };
            ListItem::new(Line::from(vec![
                Span::styled(
                    format!("{:<16}", s.name),
                    Style::default().fg(s.color),
                ),
                Span::styled(val, Style::default().fg(s.color)),
            ]))
        })
        .collect();

    let list = List::new(rows)
        .block(
            Block::default()
                .borders(Borders::ALL)
                .border_style(border_style)
                .title("Values"),
        )
        .highlight_style(
            Style::default()
                .bg(Color::DarkGray)
                .fg(Color::White)
                .add_modifier(Modifier::BOLD),
        );

    f.render_stateful_widget(list, cols[1], &mut graph.legend);
}

fn draw(f: &mut Frame, dashboard: &mut Dashboard, state: &TelemetryState) {
    let size = f.area();

    // Top-level layout: status bar (1), graphs area, bottom help bar (1), damage panel
    let main_rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(1),  // status bar
            Constraint::Min(10),   // graphs
            Constraint::Length(1),  // help bar
        ])
        .split(size);

    // Status bar
    render_status_bar(f, main_rows[0], state);

    // Graphs area: split into main charts + right sidebar (damage)
    let has_damage = state.damage.is_some();
    let content_area = if has_damage {
        // +2 for border
        let sidebar_w = MINI_DRONE_WIDTH + 2;
        let cols = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([
                Constraint::Min(40),
                Constraint::Length(sidebar_w),
            ])
            .split(main_rows[1]);

        // Tight damage panel: only as tall as needed, top-aligned
        // +2 border + 1 status + MINI_DRONE_HEIGHT
        let panel_h = (MINI_DRONE_HEIGHT + 3).min(cols[1].height);
        let damage_rect = Rect::new(cols[1].x, cols[1].y, cols[1].width, panel_h);
        render_damage_mini(f, damage_rect, state);

        cols[0]
    } else {
        main_rows[1]
    };

    // Stack graphs vertically
    let n = dashboard.graphs.len();
    let constraints: Vec<Constraint> = (0..n)
        .map(|i| {
            if i == dashboard.current_graph {
                Constraint::Ratio(2, (n + 1) as u32)
            } else {
                Constraint::Ratio(1, (n + 1) as u32)
            }
        })
        .collect();

    let graph_areas = Layout::default()
        .direction(Direction::Vertical)
        .constraints(constraints)
        .split(content_area);

    for (i, area) in graph_areas.iter().enumerate() {
        let active = i == dashboard.current_graph;
        draw_graph(f, *area, &mut dashboard.graphs[i], active);
    }

    // Help bar
    let help = Paragraph::new(Line::from(vec![
        Span::styled(" Tab", Style::default().fg(Color::Cyan)),
        Span::raw(": graph  "),
        Span::styled("↑↓", Style::default().fg(Color::Cyan)),
        Span::raw(": series  "),
        Span::styled("+/-", Style::default().fg(Color::Cyan)),
        Span::raw(": zoom  "),
        Span::styled("Esc", Style::default().fg(Color::Cyan)),
        Span::raw(": deselect  "),
        Span::styled("q", Style::default().fg(Color::Cyan)),
        Span::raw(": quit"),
    ]));
    f.render_widget(help, main_rows[2]);
}

// ---------------------------------------------------------------------------
// Zenoh subscriber tasks
// ---------------------------------------------------------------------------

fn process_crsf_frame(payload: &[u8], state: &Arc<RwLock<TelemetryState>>) {
    let Some(pkt) = crsf::parse_packet(payload) else {
        return;
    };
    let Ok(mut st) = state.write() else {
        return;
    };
    st.connected = true;
    match pkt {
        CrsfPacket::Gps(gps) => {
            st.altitude = Some(gps.alt_m());
            st.ground_speed = Some(gps.speed_kmh());
            st.heading = Some(gps.heading_deg());
            st.sats = Some(gps.sats);
        }
        CrsfPacket::Vario(vario) => {
            st.vario = Some(vario.vertical_speed_ms());
        }
        CrsfPacket::Battery(bat) => {
            st.voltage = Some(bat.voltage_v());
            st.current = Some(bat.current_a());
            st.battery_pct = Some(bat.remaining as f64);
        }
        CrsfPacket::Attitude(att) => {
            let (p, r, y) = att.as_radians();
            st.pitch = Some(p.to_degrees());
            st.roll = Some(r.to_degrees());
            st.yaw = Some(y.to_degrees());
        }
        CrsfPacket::Airspeed(air) => {
            st.airspeed = Some(air.speed_kmh());
        }
        CrsfPacket::Rpm(rpm) => {
            st.rpms = Some(rpm.rpms);
        }
        _ => {}
    }
}

fn process_damage(payload: &[u8], state: &Arc<RwLock<TelemetryState>>) {
    let Ok(pkt) = simstate::parse_damage(payload) else {
        return;
    };
    let Ok(mut st) = state.write() else {
        return;
    };
    st.connected = true;
    let mut dmg = [1.0_f32; 4];
    for (i, val) in pkt.damage.iter().enumerate().take(4) {
        dmg[i] = val.clamp(0.0, 1.0);
    }
    st.damage = Some(dmg);
    st.damage_killed = pkt.killed();
    st.damage_crashed = pkt.crashed();
    st.damage_no_drone = pkt.no_drone();
}

fn process_battery(payload: &[u8], state: &Arc<RwLock<TelemetryState>>) {
    let Ok(pkt) = simstate::parse_battery(payload) else {
        return;
    };
    let Ok(mut st) = state.write() else {
        return;
    };
    st.connected = true;
    if pkt.has_data() {
        // Prefer the more detailed simstate battery data when available.
        st.voltage = Some(pkt.voltage as f64);
        st.current = Some(pkt.current_amps as f64);
        st.battery_pct = Some(pkt.percentage as f64 * 100.0);
    }
    st.sim_battery = Some(pkt);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let state = Arc::new(RwLock::new(TelemetryState::default()));

    // Subscribe to CRSF telemetry
    {
        let topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
        info!("Subscribing to: {}", topic);
        let subscriber = session.declare_subscriber(&topic).await?;
        let state = state.clone();
        tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        process_crsf_frame(&payload, &state);
                    }
                    Err(e) => {
                        warn!("CRSF telemetry subscriber error: {}", e);
                        break;
                    }
                }
            }
        });
    }

    // Subscribe to damage
    {
        let topic = topics::topic(&args.zenoh_prefix, topics::DAMAGE);
        info!("Subscribing to: {}", topic);
        let subscriber = session.declare_subscriber(&topic).await?;
        let state = state.clone();
        tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        process_damage(&payload, &state);
                    }
                    Err(e) => {
                        warn!("Damage subscriber error: {}", e);
                        break;
                    }
                }
            }
        });
    }

    // Subscribe to battery (simstate)
    {
        let topic = topics::topic(&args.zenoh_prefix, topics::BATTERY);
        info!("Subscribing to: {}", topic);
        let subscriber = session.declare_subscriber(&topic).await?;
        let state = state.clone();
        tokio::spawn(async move {
            loop {
                match subscriber.recv_async().await {
                    Ok(sample) => {
                        let payload = sample.payload().to_bytes();
                        process_battery(&payload, &state);
                    }
                    Err(e) => {
                        warn!("Battery subscriber error: {}", e);
                        break;
                    }
                }
            }
        });
    }

    // Terminal UI
    let mut terminal = ratatui::init();
    let result = run_tui(&mut terminal, state);
    ratatui::restore();
    result
}

fn run_tui(
    terminal: &mut DefaultTerminal,
    state: Arc<RwLock<TelemetryState>>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let mut dashboard = Dashboard::new();
    let tick = Duration::from_millis(100); // ~10 Hz refresh

    loop {
        // Snapshot state and ingest into graphs
        let current = state.read().unwrap().clone();
        dashboard.ingest(&current);

        terminal.draw(|f| draw(f, &mut dashboard, &current))?;

        if event::poll(tick)?
            && let Event::Key(key) = event::read()?
        {
            if key.kind != KeyEventKind::Press {
                continue;
            }
            match key.code {
                KeyCode::Char('q') => return Ok(()),
                KeyCode::Char('l') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                    terminal.clear()?;
                }
                KeyCode::Tab => dashboard.tab_next(),
                KeyCode::BackTab => dashboard.tab_prev(),
                KeyCode::Up => {
                    dashboard.graphs[dashboard.current_graph].select_prev();
                }
                KeyCode::Down => {
                    dashboard.graphs[dashboard.current_graph].select_next();
                }
                KeyCode::Esc => {
                    dashboard.graphs[dashboard.current_graph].unselect();
                }
                KeyCode::Char('+') | KeyCode::Char('=') => {
                    for g in &mut dashboard.graphs {
                        g.zoom_in();
                    }
                }
                KeyCode::Char('-') => {
                    for g in &mut dashboard.graphs {
                        g.zoom_out();
                    }
                }
                _ => {}
            }
        }
    }
}
