# quincunx

Galton board simulation.

I wanted to learn WebAssembly for a long time because this technology looks very appealing to me as it allows to write high performance web applications using one of my favorite language - Rust. 

This project is my first attempt to make something using this technology. I've spent plenty amount of time thinking about iteresting and not too much complicated idea which would however involve 3d graphics and physics simulation.

## compile and run

Ensure you have Rust installed.

1. Install cargo-web subcommand by following instructions from here https://docs.rs/crate/cargo-web
2. Type cargo web start to compile and run project
3. Type localhost:8000 in your browser, enjoy!

## progress

The project is more or less finished now and I'm actually very happy with the result. It took me 4 full days to make it work as intended.

I used [kiss3d](https://github.com/sebcrozet/kiss3d) as a 3d engine and [nphysics](https://nphysics.org/) for physics simulation. Both libraries have bunch of good code examples to get familiar with.

## see it working online in any web-browser (at least I hope so)!

https://risboo6909.org/quincunx/
