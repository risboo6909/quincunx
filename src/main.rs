mod world;

use kiss3d::light::Light;
use kiss3d::window::{State, Window};
use kiss3d::camera::{Camera, FirstPerson};
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::renderer::Renderer;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::nalgebra as na;

use world::World;


struct AppState {
    cam: FirstPerson,
    world: World<f32>,
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

    fn step(&mut self, _window: &mut Window) {

        for (col_handler, scene_node) in self.world.objects.iter_mut() {
            let pos = self.world.colliders.get(*col_handler).unwrap().position();
            scene_node.set_local_transformation(*pos);
        }

        // self.c.prepend_to_local_translation(&na::Translation3::new(0.0, -0.001, 0.0));

        self.world.mworld.step(
            &mut self.world.gworld, 
            &mut self.world.bodies, 
            &mut self.world.colliders, 
            &mut self.world.joint_constraints, 
            &mut self.world.force_generators
        );
    }
}

fn main() {

    let mut world = World::new();

    let mut window = Window::new("Quincunx");

    // camera
    let eye = na::Point3::new(1.0, 1.0, 10.0);
    let at = na::Point3::origin();
    let cam = FirstPerson::new(eye, at);
    
    let mut sphere = world.make_sphere(&mut window, 0.0, 0.0, 0.0, 0.1);
    sphere.set_color(1.0, 0.0, 0.0);

    let mut boxx = world.make_box(&mut window, 0.0, -3.5, 0.0, 1.0, 0.2, 1.0);
    boxx.set_color(0.0, 1.0, 0.0);

    window.set_light(Light::StickToCamera);

    let state = AppState {
        cam: cam,
        world,
     };

    window.render_loop(state)
}
