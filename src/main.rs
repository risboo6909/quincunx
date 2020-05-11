mod world;

use kiss3d::camera::{ArcBall, Camera};
use kiss3d::light::Light;
use kiss3d::nalgebra as na;
use kiss3d::planar_camera::PlanarCamera;
use kiss3d::post_processing::PostProcessingEffect;
use kiss3d::renderer::Renderer;
use kiss3d::window::{State, Window};

use world::World;

struct AppState {
    cam: ArcBall,
    world: World<f32>,
}

impl State for AppState {
    fn cameras_and_effect_and_renderer(
        &mut self,
    ) -> (
        Option<&mut dyn Camera>,
        Option<&mut dyn PlanarCamera>,
        Option<&mut dyn Renderer>,
        Option<&mut dyn PostProcessingEffect>,
    ) {
        (Some(&mut self.cam), None, None, None)
    }

    fn step(&mut self, _window: &mut Window) {
        for (col_handler, scene_node) in self.world.objects.iter_mut() {
            let pos = self.world.colliders.get(*col_handler).unwrap().position();
            scene_node.set_local_transformation(*pos);
        }

        self.world.mworld.step(
            &mut self.world.gworld,
            &mut self.world.bodies,
            &mut self.world.colliders,
            &mut self.world.joint_constraints,
            &mut self.world.force_generators,
        );
    }
}

fn main() {
    let mut world = World::new();

    let mut window = Window::new("Quincunx");

    // camera
    let eye = na::Point3::new(0.0, 0.0, 18.0);
    let at = na::Point3::new(0.0, 0.0, -0.5);

    let mut cam = ArcBall::new(eye, at);
    cam.set_pitch(-90.0);

    world.make_board(&mut window, 0.0, 0.0, -0.4);

    window.set_light(Light::StickToCamera);

    let mut state = AppState { cam: cam, world };

    state.step(&mut window);

    window.render_loop(state)
}
