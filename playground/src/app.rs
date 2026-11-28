//! The playground's window: the scene on a canvas, and a side panel with the query, its
//! answers, and the selected shape.

use crate::scene::{Answer, Body, Form, MAX_SIDES, Query, Scene};
use auburn2d::Vec2;
use auburn2d::transformation::Transformation2d;
use eframe::egui::{
    self, Align2, Color32, CursorIcon, DragValue, FontId, Pos2, Rect, Sense, Stroke,
};

const MOVING: Color32 = Color32::from_rgb(255, 165, 70);
const OTHER: Color32 = Color32::from_rgb(110, 170, 255);
const ENTRY: Color32 = Color32::from_rgb(90, 210, 120);
const EXIT: Color32 = Color32::from_rgb(240, 95, 95);
const PUSH: Color32 = Color32::from_rgb(205, 135, 255);
const HANDLE: Color32 = Color32::from_gray(220);
/// How near the pointer has to be, in points, to grab a handle or a point.
const GRAB: f32 = 8.0;
/// What Shift snaps positions and scales to, in world units, and angles to.
const SNAP: f32 = 0.25;
const SNAP_ANGLE: f32 = std::f32::consts::PI / 12.0;

/// What the canvas shows: the world point at its centre, and how many points a world unit is.
#[derive(Clone, Copy)]
struct View {
    center: Vec2,
    zoom: f32,
}

impl Default for View {
    fn default() -> Self {
        Self {
            center: Vec2::ZERO,
            zoom: 60.0,
        }
    }
}

impl View {
    fn screen(self, rect: Rect, p: Vec2) -> Pos2 {
        rect.center() + self.screen_vec(p - self.center)
    }

    fn screen_vec(self, v: Vec2) -> egui::Vec2 {
        egui::vec2(v.x, -v.y) * self.zoom
    }

    fn world(self, rect: Rect, p: Pos2) -> Vec2 {
        let d = (p - rect.center()) / self.zoom;
        self.center + Vec2::new(d.x, -d.y)
    }

    /// Zooms in by `factor`, keeping the world point under `p` where it is.
    fn zoom_at(&mut self, rect: Rect, p: Pos2, factor: f32) {
        let fixed = self.world(rect, p);
        self.zoom = (self.zoom * factor).clamp(1e-3, 1e6);
        self.center += fixed - self.world(rect, p);
    }
}

/// What a drag on the canvas moves.
#[derive(Clone, Copy)]
enum Grab {
    /// A body, held this far from its origin.
    Body(usize, Vec2),
    /// A body's turn-and-scale handle.
    Handle(usize),
    /// The tip of the moving body's velocity.
    Velocity,
    View,
}

pub struct App {
    scene: Scene,
    view: View,
    /// The body the side panel edits, which shows its handle.
    selected: usize,
    grab: Option<Grab>,
    /// The scene as text, in the side panel.
    text: String,
    /// The target whose test was copied last, and the scene it was copied from.
    copied: Option<(usize, Scene)>,
    /// The page's URL after the `#`, as last read or written.
    #[cfg(target_arch = "wasm32")]
    hash: String,
}

impl App {
    pub fn new(scene: Option<Scene>) -> Self {
        let scene = scene.unwrap_or_default();
        Self {
            selected: scene.moving,
            text: scene.to_text(),
            scene,
            view: View::default(),
            grab: None,
            copied: None,
            #[cfg(target_arch = "wasm32")]
            hash: String::new(),
        }
    }

    /// The bodies in the order they're drawn: the moving one on top.
    fn draw_order(&self) -> impl DoubleEndedIterator<Item = usize> + use<> {
        self.scene.others().chain([self.scene.moving])
    }

    fn outline(&self, rect: Rect, i: usize, offset: Vec2) -> Outline {
        Outline::of(self.view, rect, &self.scene.bodies[i], offset)
    }

    /// The body under `p`, the topmost if several are.
    fn body_at(&self, rect: Rect, p: Pos2) -> Option<usize> {
        let mut order = self.draw_order().rev();
        order.find(|&i| self.outline(rect, i, Vec2::ZERO).contains(p))
    }

    /// What a drag from `p` would move. A handle that sits on its body's origin (when the
    /// velocity is short, say) gives way to the body.
    fn grab_at(&self, rect: Rect, p: Pos2) -> Grab {
        let scene = &self.scene;
        let near = |w: Vec2| self.view.screen(rect, w).distance(p) <= GRAB;
        let apart = |w: Vec2, origin: Vec2| {
            self.view
                .screen(rect, w)
                .distance(self.view.screen(rect, origin))
                > GRAB
        };
        let moving = &scene.bodies[scene.moving];
        let tip = moving.pos + scene.vel;
        let on_tip = scene.query.is_cast() && near(tip);
        let selected = &scene.bodies[self.selected];
        let on_handle = near(handle(selected));
        if on_tip && apart(tip, moving.pos) {
            Grab::Velocity
        } else if on_handle && apart(handle(selected), selected.pos) {
            Grab::Handle(self.selected)
        } else if let Some(i) = self.body_at(rect, p) {
            Grab::Body(i, scene.bodies[i].pos - self.view.world(rect, p))
        } else if on_tip {
            Grab::Velocity
        } else if on_handle {
            Grab::Handle(self.selected)
        } else {
            Grab::View
        }
    }

    fn side_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("auburn playground");
        ui.weak(format!("auburn2d {}", env!("CARGO_PKG_VERSION")));
        ui.separator();

        ui.strong("Query");
        ui.horizontal_wrapped(|ui| {
            for query in Query::ALL {
                let name = egui::RichText::new(query.name()).monospace();
                ui.selectable_value(&mut self.scene.query, query, name);
            }
        });
        ui.label(self.scene.query.about());
        ui.add_space(4.0);
        ui.horizontal(|ui| {
            ui.label("Moving");
            for (i, body) in self.scene.bodies.iter().enumerate() {
                ui.selectable_value(&mut self.scene.moving, i, body.form.name());
            }
        });
        if self.scene.query.is_cast() {
            ui.horizontal(|ui| {
                ui.label("Velocity");
                ui.add(DragValue::new(&mut self.scene.vel.x).speed(0.01));
                ui.add(DragValue::new(&mut self.scene.vel.y).speed(0.01));
            });
        }
        ui.separator();

        ui.strong("Answers");
        let moving = self.scene.bodies[self.scene.moving].form.name();
        for target in self.scene.others() {
            let answer = self.scene.answer(target);
            let name = self.scene.bodies[target].form.name();
            let query = self.scene.query.name();
            ui.horizontal(|ui| {
                ui.monospace(format!("{moving}.{query}({name})"));
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let copied = self.copied.as_ref() == Some(&(target, self.scene.clone()));
                    let button = ui.small_button(if copied { "Copied" } else { "Copy as test" });
                    let button = button.on_hover_text(
                        "Copies a Rust test that asserts this answer, for \
                         auburn2d/tests/regressions.rs.",
                    );
                    if button.clicked() {
                        ui.ctx().copy_text(self.scene.test_code(target));
                        self.copied = Some((target, self.scene.clone()));
                    }
                });
            });
            let color = if answer.hit() {
                OTHER
            } else {
                ui.visuals().text_color()
            };
            let text = egui::RichText::new(answer.to_string())
                .monospace()
                .color(color);
            ui.add(egui::Label::new(text).wrap());
            ui.add_space(2.0);
        }
        ui.separator();

        ui.strong("Shape");
        ui.horizontal(|ui| {
            for (i, body) in self.scene.bodies.iter().enumerate() {
                ui.selectable_value(&mut self.selected, i, body.form.name());
            }
        });
        let body = &mut self.scene.bodies[self.selected];
        egui::Grid::new("shape").num_columns(2).show(ui, |ui| {
            ui.label("Position");
            ui.horizontal(|ui| {
                ui.add(DragValue::new(&mut body.pos.x).speed(0.01));
                ui.add(DragValue::new(&mut body.pos.y).speed(0.01));
            });
            ui.end_row();
            ui.label("Angle");
            let mut degrees = body.angle.to_degrees();
            let angle = DragValue::new(&mut degrees).speed(0.5).suffix("°");
            if ui.add(angle).changed() {
                body.angle = degrees.to_radians();
            }
            ui.end_row();
            ui.label("Scale");
            ui.add(DragValue::new(&mut body.scale).speed(0.01));
            ui.end_row();
            match &mut body.form {
                Form::Point => {}
                Form::Ball { radius } => {
                    ui.label("Radius");
                    ui.add(size(radius));
                    ui.end_row();
                }
                Form::Box { half_size } => {
                    ui.label("Half size");
                    ui.horizontal(|ui| {
                        ui.add(size(&mut half_size.x));
                        ui.add(size(&mut half_size.y));
                    });
                    ui.end_row();
                }
                Form::Polygon { sides, radius } => {
                    ui.label("Sides");
                    ui.add(DragValue::new(sides).speed(0.05).range(0..=MAX_SIDES));
                    ui.end_row();
                    ui.label("Radius");
                    ui.add(size(radius));
                    ui.end_row();
                }
            }
        });
        ui.separator();

        ui.strong("Scene");
        ui.label("Paste a playground link here to open it.");
        let id = ui.make_persistent_id("scene text");
        if !ui.memory(|memory| memory.has_focus(id)) {
            self.text = self.scene.to_text();
        }
        let edit = egui::TextEdit::multiline(&mut self.text)
            .id(id)
            .font(egui::TextStyle::Monospace)
            .desired_rows(3)
            .desired_width(f32::INFINITY);
        let response = ui.add(edit);
        let read = Scene::from_link(&self.text);
        if response.changed()
            && let Some(scene) = &read
        {
            self.scene = scene.clone();
        }
        if response.has_focus() && read.is_none() {
            ui.colored_label(EXIT, "That isn't a playground scene.");
        }
        if ui.button("Reset").clicked() {
            self.scene = Scene::default();
            self.view = View::default();
            self.selected = self.scene.moving;
        }
        ui.separator();

        egui::CollapsingHeader::new("How to use")
            .default_open(true)
            .show(ui, |ui| {
                for line in [
                    "Drag a shape to move it, or empty space to pan.",
                    "Drag the round handle to turn and scale the selected shape. Or scroll \
                     over a shape to scale it, and hold Shift to turn it instead.",
                    "Scroll elsewhere to zoom.",
                    "Drag the arrow's tip to change the velocity.",
                    "Hold Shift while dragging to snap.",
                    "Double-click a shape to make it the moving one.",
                    "Ghosts show the moving shape when it enters (green) and exits (red), \
                     or pushed out (purple).",
                    "A cast's hit shows where the shapes touch, and the normal out of the \
                     other shape there (purple).",
                ] {
                    ui.label(line);
                }
            });
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (response, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let before = self.scene.clone();
        self.interact(ui, &response);
        if self.scene != before {
            // The side panel has shown the scene from before already.
            ui.ctx().request_repaint();
        }
        let hovered = response
            .hover_pos()
            .and_then(|p| self.body_at(response.rect, p));
        self.paint(&painter, response.rect, hovered);
    }

    fn interact(&mut self, ui: &egui::Ui, response: &egui::Response) {
        let rect = response.rect;
        let (shift, scroll, zoom) =
            ui.input(|i| (i.modifiers.shift, i.smooth_scroll_delta, i.zoom_delta()));

        if response.drag_started()
            && let Some(p) = ui.input(|i| i.pointer.press_origin())
        {
            let grab = self.grab_at(rect, p);
            if let Grab::Body(i, _) | Grab::Handle(i) = grab {
                self.selected = i;
            }
            self.grab = Some(grab);
        }
        if response.dragged()
            && let (Some(grab), Some(p)) = (self.grab, response.interact_pointer_pos())
        {
            let w = self.view.world(rect, p);
            let snap = |v: Vec2| if shift { (v / SNAP).round() * SNAP } else { v };
            let scene = &mut self.scene;
            match grab {
                Grab::Body(i, offset) => scene.bodies[i].pos = snap(w + offset),
                Grab::Handle(i) => turn_and_scale(&mut scene.bodies[i], w, shift),
                Grab::Velocity => scene.vel = snap(w - scene.bodies[scene.moving].pos),
                Grab::View => {
                    let d = response.drag_delta() / self.view.zoom;
                    self.view.center -= Vec2::new(d.x, -d.y);
                }
            }
        }
        if response.drag_stopped() {
            self.grab = None;
        }

        let Some(p) = response.hover_pos() else {
            return;
        };
        let hovered = self.body_at(rect, p);
        if let Some(i) = hovered {
            if response.clicked() {
                self.selected = i;
            }
            if response.double_clicked() {
                self.scene.moving = i;
            }
        }
        let amount = scroll.x + scroll.y;
        if amount != 0.0 {
            match hovered {
                Some(i) if shift => self.scene.bodies[i].angle += amount * 0.004,
                Some(i) => self.scene.bodies[i].scale *= (amount * 0.003).exp(),
                None => self.view.zoom_at(rect, p, (amount * 0.003).exp()),
            }
        }
        if zoom != 1.0 {
            self.view.zoom_at(rect, p, zoom);
        }
        let cursor = match (self.grab, self.grab_at(rect, p)) {
            (Some(_), _) => CursorIcon::Grabbing,
            (None, Grab::View) => CursorIcon::Default,
            (None, _) => CursorIcon::Grab,
        };
        ui.ctx().set_cursor_icon(cursor);
    }

    fn paint(&self, painter: &egui::Painter, rect: Rect, hovered: Option<usize>) {
        self.paint_grid(painter, rect);
        let scene = &self.scene;
        let answers: Vec<(usize, Answer)> = scene.others().map(|i| (i, scene.answer(i))).collect();
        let hit = |i: usize| {
            let mut hits = answers.iter().filter(|(_, answer)| answer.hit());
            hits.any(|&(target, _)| i == scene.moving || i == target)
        };
        for i in self.draw_order() {
            let color = if i == scene.moving { MOVING } else { OTHER };
            let width = if Some(i) == hovered || i == self.selected {
                2.5
            } else {
                1.5
            };
            let outline = self.outline(rect, i, Vec2::ZERO);
            outline.paint(painter, Stroke::new(width, color), hit(i));
            let origin = self.view.screen(rect, scene.bodies[i].pos);
            let cross = Stroke::new(1.0, color);
            painter.line_segment(
                [origin - egui::vec2(3.0, 0.0), origin + egui::vec2(3.0, 0.0)],
                cross,
            );
            painter.line_segment(
                [origin - egui::vec2(0.0, 3.0), origin + egui::vec2(0.0, 3.0)],
                cross,
            );
        }

        let moving = &scene.bodies[scene.moving];
        let origin = self.view.screen(rect, moving.pos);
        let ghost = |offset: Vec2, color: Color32, time: Option<f32>| {
            if !offset.is_finite() {
                return;
            }
            let outline = self.outline(rect, scene.moving, offset);
            outline.paint_dashed(painter, Stroke::new(1.5, color));
            if let Some(t) = time {
                let at = self.view.screen(rect, moving.pos + offset) - egui::vec2(0.0, 14.0);
                let font = FontId::proportional(12.0);
                painter.text(at, Align2::CENTER_CENTER, format!("{t:.3}"), font, color);
            }
        };
        // Under the answers, so that it doesn't hide a hit's normal.
        if scene.query.is_cast() {
            painter.arrow(
                origin,
                self.view.screen_vec(scene.vel),
                Stroke::new(2.0, MOVING),
            );
            let tip = self.view.screen(rect, moving.pos + scene.vel);
            painter.circle_filled(tip, 5.0, MOVING);
        }
        for (_, answer) in &answers {
            match *answer {
                Answer::Penetrates(Some(push)) => {
                    ghost(push, PUSH, None);
                    painter.arrow(origin, self.view.screen_vec(push), Stroke::new(2.0, PUSH));
                }
                Answer::Time(Some(t)) => ghost(t * scene.vel, ENTRY, Some(t)),
                Answer::Span(Some((entry, exit))) => {
                    ghost(entry * scene.vel, ENTRY, Some(entry));
                    ghost(exit * scene.vel, EXIT, Some(exit));
                }
                Answer::Hit(Some(hit)) => {
                    ghost(hit.time * scene.vel, ENTRY, Some(hit.time));
                    // The normal at a length that doesn't depend on the zoom.
                    let at = self.view.screen(rect, hit.point);
                    let normal = self.view.screen_vec(hit.normal).normalized() * 36.0;
                    painter.arrow(at, normal, Stroke::new(2.0, PUSH));
                    painter.circle_filled(at, 4.0, PUSH);
                }
                _ => {}
            }
        }

        let selected = &scene.bodies[self.selected];
        let (from, to) = (
            self.view.screen(rect, selected.pos),
            self.view.screen(rect, handle(selected)),
        );
        painter.line_segment([from, to], Stroke::new(1.0, HANDLE.gamma_multiply(0.5)));
        painter.circle(to, 5.0, Color32::TRANSPARENT, Stroke::new(1.5, HANDLE));
    }

    /// Lines a power of ten world units apart, at least 24 points apart, and the axes.
    fn paint_grid(&self, painter: &egui::Painter, rect: Rect) {
        let view = self.view;
        let step = 10f32.powf((24.0 / view.zoom).log10().ceil());
        let (lo, hi) = (
            view.world(rect, rect.left_bottom()),
            view.world(rect, rect.right_top()),
        );
        let line = |k: i64| {
            let gray = if k == 0 { 90 } else { 45 };
            Stroke::new(1.0, Color32::from_gray(gray))
        };
        let steps = |lo: f32, hi: f32| (lo / step).floor() as i64..=(hi / step).ceil() as i64;
        for k in steps(lo.x, hi.x) {
            let x = view.screen(rect, Vec2::new(k as f32 * step, 0.0)).x;
            painter.vline(x, rect.y_range(), line(k));
        }
        for k in steps(lo.y, hi.y) {
            let y = view.screen(rect, Vec2::new(0.0, k as f32 * step)).y;
            painter.hline(rect.x_range(), y, line(k));
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn read_hash(&mut self) {
        let hash = crate::web::hash();
        if hash != self.hash {
            if let Some(scene) = Scene::from_text(&hash) {
                self.selected = scene.moving;
                self.scene = scene;
            }
            self.hash = hash;
        }
    }

    #[cfg(target_arch = "wasm32")]
    fn write_hash(&mut self, ctx: &egui::Context) {
        let hash = self.scene.to_text();
        if hash != self.hash {
            crate::web::set_hash(&hash);
            self.hash = hash;
        }
        // Someone may edit the address bar: look again in a while.
        ctx.request_repaint_after(std::time::Duration::from_millis(500));
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        #[cfg(target_arch = "wasm32")]
        self.read_hash();
        egui::Panel::left("side")
            .default_size(330.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| self.side_panel(ui))
            });
        let canvas = egui::Frame::NONE.fill(Color32::from_gray(24));
        egui::CentralPanel::default()
            .frame(canvas)
            .show(ui, |ui| self.canvas(ui));
        #[cfg(target_arch = "wasm32")]
        self.write_hash(ui.ctx());
    }
}

/// Where `body`'s turn-and-scale handle is.
fn handle(body: &Body) -> Vec2 {
    body.transform().apply(Vec2::new(body.form.reach(), 0.0))
}

/// Turns and scales `body` about its origin, so that its handle is at `w`. A negative scale
/// stays negative.
fn turn_and_scale(body: &mut Body, w: Vec2, snap: bool) {
    let sign = if body.scale < 0.0 { -1.0 } else { 1.0 };
    let d = sign * (w - body.pos);
    let (mut angle, mut scale) = (d.y.atan2(d.x), d.length() / body.form.reach());
    if snap {
        angle = (angle / SNAP_ANGLE).round() * SNAP_ANGLE;
        scale = ((scale / SNAP).round() * SNAP).max(SNAP);
    }
    body.angle = angle;
    body.scale = sign * scale;
}

/// A body as the canvas draws it, in screen points.
enum Outline {
    Dot(Pos2),
    /// A ball's centre and radius, and where its radius at angle 0 ends, which shows its turn.
    Circle(Pos2, f32, Pos2),
    /// A box's or a polygon's vertices.
    Polygon(Vec<Pos2>),
}

impl Outline {
    fn of(view: View, rect: Rect, body: &Body, offset: Vec2) -> Self {
        let t = Body {
            pos: body.pos + offset,
            ..*body
        }
        .transform();
        let screen = |p: Vec2| view.screen(rect, t.apply(p));
        match body.form {
            Form::Point => Self::Dot(screen(Vec2::ZERO)),
            Form::Ball { radius } => Self::Circle(
                screen(Vec2::ZERO),
                (radius * body.scale).abs() * view.zoom,
                screen(Vec2::new(radius, 0.0)),
            ),
            form => match form.vertices() {
                points if points.is_empty() => Self::Dot(screen(Vec2::ZERO)),
                points => Self::Polygon(points.into_iter().map(screen).collect()),
            },
        }
    }

    /// Whether `p` is on it, or within [`GRAB`] of its edge.
    fn contains(&self, p: Pos2) -> bool {
        match self {
            Self::Dot(c) => c.distance(p) <= GRAB,
            Self::Circle(c, r, _) => c.distance(p) <= r + GRAB,
            Self::Polygon(points) => {
                let n = points.len();
                let edges = || (0..n).map(|i| (points[i], points[(i + 1) % n]));
                let turns = edges().map(|(a, b)| (b - a).x * (p - a).y - (b - a).y * (p - a).x);
                let (left, right) = turns.fold((false, false), |(l, r), turn| {
                    (l || turn > 0.0, r || turn < 0.0)
                });
                (n >= 3 && left != right) || edges().any(|(a, b)| distance(p, a, b) <= GRAB)
            }
        }
    }

    /// Draws it, filled if `hit`.
    fn paint(&self, painter: &egui::Painter, stroke: Stroke, hit: bool) {
        let fill = if hit {
            stroke.color.gamma_multiply(0.35)
        } else {
            Color32::TRANSPARENT
        };
        match self {
            Self::Dot(c) => {
                let fill = if hit {
                    stroke.color
                } else {
                    Color32::TRANSPARENT
                };
                painter.circle(*c, 4.0, fill, stroke);
            }
            Self::Circle(c, r, spoke) => {
                painter.circle(*c, *r, fill, stroke);
                painter.line_segment([*c, *spoke], stroke);
            }
            Self::Polygon(points) if points.len() >= 3 => {
                painter.add(egui::Shape::convex_polygon(points.clone(), fill, stroke));
            }
            Self::Polygon(points) => {
                painter.add(egui::Shape::line(points.clone(), stroke));
                for p in points {
                    painter.circle_filled(*p, 3.0, stroke.color);
                }
            }
        }
    }

    fn paint_dashed(&self, painter: &egui::Painter, stroke: Stroke) {
        let path: Vec<Pos2> = match self {
            Self::Dot(c) => {
                painter.circle_stroke(*c, 4.0, stroke);
                return;
            }
            Self::Circle(c, r, _) => (0..=64)
                .map(|k| *c + *r * egui::Vec2::angled(k as f32 * std::f32::consts::TAU / 64.0))
                .collect(),
            Self::Polygon(points) => points.iter().chain(points.first()).copied().collect(),
        };
        painter.extend(egui::Shape::dashed_line(&path, stroke, 6.0, 4.0));
    }
}

/// How far `p` is from the segment from `a` to `b`.
fn distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    let t = if ab.length_sq() > 0.0 {
        ((p - a).dot(ab) / ab.length_sq()).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (a + t * ab).distance(p)
}

/// A drag value for a size, which can't go below 0.
fn size(value: &mut f32) -> DragValue<'_> {
    DragValue::new(value).speed(0.01).range(0.0..=f32::MAX)
}
