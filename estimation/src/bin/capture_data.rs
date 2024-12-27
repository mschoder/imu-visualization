use anyhow::{Context, Result};
use clap::{arg, command, Parser};
use serialport::{DataBits, FlowControl, Parity, StopBits};
use std::fs::OpenOptions;
use std::io::{Read, Write};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

const HEADER_BYTES: [u8; 2] = [0xAA, 0xFF]; // Header bytes
const HEADER_COLS: &str = "time,ax,ay,az,gx,gy,gz,mx,my,mz";
const STATE_VECTOR_SIZE: usize = 36; // 9 floats * 4 bytes each

#[derive(Clone, Parser, Debug)]
#[command(version, about, long_about = None)]
struct CliArgs {
    /// Serial port to read input data from
    #[arg(
        short,
        long,
        conflicts_with = "file",
        default_value = "/dev/cu.usbmodem1101"
    )]
    port: String,
    /// Baud rate for the serial port
    #[arg(short, long, default_value = "115200")]
    baud: u32,
    /// File to write raw IMU data to.
    #[arg(
        short,
        long,
        conflicts_with = "port",
        default_value = "raw_imu_data.csv"
    )]
    file_path: String,
    /// Overwrite the output file if it exists
    #[arg(short, long)]
    overwrite: bool,
}

fn main() -> Result<()> {
    let args = CliArgs::parse();

    // Create serial port connection
    let mut port = serialport::new(&args.port, args.baud)
        .data_bits(DataBits::Eight)
        .parity(Parity::None)
        .stop_bits(StopBits::One)
        .flow_control(FlowControl::None)
        .timeout(Duration::from_millis(3000))
        .open()
        .with_context(|| format!("Failed to open serial port {}", args.port))?;

    if args.overwrite {
        if std::path::Path::new(&args.file_path).exists() {
            std::fs::remove_file(&args.file_path)
                .with_context(|| format!("Failed to delete existing file: {}", args.file_path))?;
        }
    }

    // Open or create the output file for appending
    let mut file = OpenOptions::new()
        .create(true)
        .append(true)
        .open(&args.file_path)
        .with_context(|| format!("Failed to open filepath: {}", args.file_path))?;

    // Write the header to the file if it's empty
    let metadata = file.metadata()?;
    if metadata.len() == 0 {
        writeln!(file, "{HEADER_COLS}")?;
    }

    let mut buffer = [0u8; 1]; // Buffer to read one byte at a time
    let mut header_buffer = Vec::new(); // Buffer to detect the 2-byte header
    let mut message_buffer = [0u8; STATE_VECTOR_SIZE];

    let start = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    let clock = std::time::Instant::now();

    loop {
        // Read one byte
        match port.read_exact(&mut buffer) {
            Ok(_) => {
                header_buffer.push(buffer[0]);

                // Keep only the last two bytes in the buffer
                if header_buffer.len() > HEADER_BYTES.len() {
                    header_buffer.remove(0);
                }

                // Check for the 2-byte header 0xAA, 0xFF
                if header_buffer == HEADER_BYTES {
                    // Read the 9-byte message sequence
                    match port.read_exact(&mut message_buffer) {
                        Ok(_) => {
                            // Convert the bytes into 9 floats
                            let state_vector: Vec<f32> = message_buffer
                                .chunks(4)
                                .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
                                .collect();

                            // Append the state vector to the file
                            let time = start + clock.elapsed();
                            let values = state_vector
                                .iter()
                                .map(|value| value.to_string())
                                .collect::<Vec<String>>()
                                .join(",");
                            writeln!(file, "{},{}", time.as_secs_f64(), values)?;

                            // Ensure the data is written to disk
                            file.flush()?;
                        }
                        Err(e) => {
                            eprintln!("Error reading message: {}", e);
                        }
                    }

                    // Clear the header buffer to look for the next header
                    header_buffer.clear();
                }
            }
            Err(e) => {
                eprintln!("Read error: {}", e);
                header_buffer.clear();
                continue;
            }
        }
    }
}
