fn main() {
    if cfg!(windows) {
        let mut resource = winres::WindowsResource::new();
        resource.set_icon("assets\\foveated_lidar.ico");
        resource
            .compile()
            .expect("failed to compile Windows icon resource");
    }
}
