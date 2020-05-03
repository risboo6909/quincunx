use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use kiss3d::window::{State, Window};
use nalgebra::{Translation3};

struct AppState {
    c: SceneNode,
    //rot: UnitQuaternion<f32>,
}

impl State for AppState {
    fn step(&mut self, _: &mut Window) {
        self.c.prepend_to_local_translation(&Translation3::new(0.0, -0.001, 0.0));
    }
}

fn main() {
    let mut window = Window::new("Quincunx");
    let mut c = window.add_sphere(0.01);

    c.set_color(1.0, 0.0, 0.0);

    window.set_light(Light::StickToCamera);
    let state = AppState { c };

    window.render_loop(state)
}
