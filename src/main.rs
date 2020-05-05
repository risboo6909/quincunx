use kiss3d::scene::SceneNode;
use kiss3d::light::Light;
use kiss3d::window::{State, Window};
use kiss3d::event::{Action, WindowEvent};
use kiss3d::camera::{Camera, ArcBall};
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::renderer::Renderer;
use kiss3d::post_processing::PostProcessingEffect;

use nalgebra::{Translation3, Point3};

struct AppState {
    cam: ArcBall,
    c: SceneNode,
    //rot: UnitQuaternion<f32>,
}

impl State for AppState {

    fn cameras_and_effect_and_renderer(&mut self) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    )  {
        (Some(&mut self.cam), None, None, None)
    }

    fn step(&mut self, window: &mut Window) {

        for event in window.events().iter() {
            match event.value {
                WindowEvent::MouseButton(button, Action::Press, modif) => {
                    // println!("mouse press event on {:?} with {:?}", button, modif);
                    //let window_size = nalgebra::Vector2::new(window.size()[0] as f32, window.size()[1] as f32);
                    //sel_pos = camera.unproject(&last_pos, &window_size);
                    // println!(
                    //     "conv {:?} to {:?} win siz {:?} ",
                    //     last_pos, sel_pos, window_size
                    // );

                }
                _ => {}
            }
        }

        self.c.prepend_to_local_translation(&Translation3::new(0.0, -0.001, 0.0));
    }
}

fn main() {
    let eye = Point3::new(1f32, 1.0, 1.0);
    let at = Point3::origin();

    let mut window = Window::new("Quincunx");

    let arc_ball = ArcBall::new(eye, at);
    let mut c = window.add_sphere(0.01);

    c.set_color(1.0, 0.0, 0.0);
    c.set_local_translation(Translation3::new(0.0, 0.0, 0.0));

    window.set_light(Light::StickToCamera);
    let state = AppState { c, cam: arc_ball };

    window.render_loop(state)
}
