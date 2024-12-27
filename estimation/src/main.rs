use anyhow::Result;
use clap::{arg, command, Parser};
use complementary::ComplementaryFilter;
use ekf::Ekf;
use futures_util::SinkExt;
use imu_filters::*;
use madgwick::Madgwick;
use std::time::{Duration, UNIX_EPOCH};
use std::{path::PathBuf, time::SystemTime};
use tokio::io::AsyncReadExt;
use tokio::net::TcpListener;
use tokio_serial::SerialPortBuilderExt;
use tokio_tungstenite::{accept_async, tungstenite::protocol::Message};

const HEADER: [u8; 2] = [0xAA, 0xFF]; // Header bytes
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
    serial: Option<String>,

    /// Serial port baud rate
    #[arg(short, long, default_value = "115200")]
    baud: u32,

    /// Specify an input file to read raw data from (instead of from serial)
    #[arg(short, long, conflicts_with = "serial")]
    file: Option<PathBuf>,

    /// Server address on which to send computed attitude estimates (via websocket)
    #[arg(long, default_value = "localhost:8089")]
    server_addr: String,

    #[arg(short, long)]
    debug: bool,
}

async fn read_attitude_from_serial(
    args: CliArgs,
    data_tx: tokio::sync::mpsc::Sender<RawImuData>,
) -> Result<()> {
    let addr = args.serial.expect("Serial address should not be null.");
    let mut port = tokio_serial::new(addr, args.baud).open_native_async()?;

    let mut buffer = [0u8; 1]; // Buffer to read one byte at a time
    let mut header_buffer = Vec::new(); // Buffer to detect the 2-byte header
    let mut message_buffer = [0u8; STATE_VECTOR_SIZE];
    let start = SystemTime::now().duration_since(UNIX_EPOCH).unwrap();
    let clock = std::time::Instant::now();

    loop {
        // Read one byte
        match port.read_exact(&mut buffer).await {
            Ok(_) => {
                header_buffer.push(buffer[0]);

                // Keep only the last two bytes in the buffer
                if header_buffer.len() > HEADER.len() {
                    header_buffer.remove(0);
                }

                // Check for the 2-byte header 0xAA, 0xFF
                if header_buffer == HEADER {
                    // Read the 36-byte message sequence
                    match port.read_exact(&mut message_buffer).await {
                        Ok(_) => {
                            // Convert the bytes into 9 floats
                            let state_vector: Vec<f32> = message_buffer
                                .chunks(4)
                                .map(|chunk| f32::from_le_bytes(chunk.try_into().unwrap()))
                                .collect();
                            let raw = RawImuData {
                                time: start + clock.elapsed(),
                                ax: state_vector[0],
                                ay: state_vector[1],
                                az: state_vector[2],
                                gx: state_vector[3],
                                gy: state_vector[4],
                                gz: state_vector[5],
                                mx: state_vector[6],
                                my: state_vector[7],
                                mz: state_vector[8],
                            };

                            if let Err(e) = data_tx.try_send(raw) {
                                eprintln!("Failed to send parsed data: {e}");
                            }
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

async fn read_attitude_from_file(file: PathBuf, data_tx: tokio::sync::mpsc::Sender<RawImuData>) {
    // Send at a fixed rate
    let mut interval = tokio::time::interval(Duration::from_millis(10));

    let data = read_csv(&file).expect("Failed to read csv.");
    let mut idx: usize = 0;

    loop {
        interval.tick().await;
        idx = (idx + 1) % data.len();

        let raw = &data[idx];
        if let Err(e) = data_tx.try_send(raw.clone()) {
            eprintln!("Failed to send parsed data: {e}");
        }
    }
}

async fn run_estimators(
    _args: CliArgs,
    mut data_rx: tokio::sync::mpsc::Receiver<RawImuData>,
    mut ws_stream: tokio_tungstenite::WebSocketStream<tokio::net::TcpStream>,
) {
    // initial state - get first data point to seed initial estimates
    let Some(raw) = data_rx.recv().await else {
        panic!("Raw IMU data rx channel closed, exiting");
    };
    let mut complementary_filter = ComplementaryFilter::new(&raw, 0.995);
    let mut extended_kalman_filter = Ekf::new(&raw);
    let mut madgwick_filter = Madgwick::new(&raw, 0.04);

    // Loop on new data update
    while let Some(raw) = data_rx.recv().await {
        // Update each estimator
        complementary_filter.update(&raw);
        extended_kalman_filter.update(&raw);
        madgwick_filter.update(&raw);

        let estimates = FilteredEstimates {
            // accel_mag: orientation_from_accel_mag(&raw),
            accel_mag: orientation_accel_mag(&raw),
            complementary: complementary_filter.estimate(),
            ekf: extended_kalman_filter.estimate(),
            madgwick: madgwick_filter.estimate(),
        };

        // Serialize attitude estimates to JSON and send over websocket
        let json_data = serde_json::to_string(&estimates).expect("Failed to serialize estimates");
        ws_stream
            .send(Message::Text(json_data.clone()))
            .await
            .expect("Failed to send message");
    }
}

async fn run_server(args: CliArgs) {
    // Bind to the address
    let listener = TcpListener::bind(&args.server_addr)
        .await
        .expect("Failed to bind to address");
    println!("WebSocket server running on {}", &args.server_addr);

    // Accept incoming WebSocket connections
    while let Ok((stream, _)) = listener.accept().await {
        // Upgrade the TCP stream to a WebSocket stream
        let ws_stream = accept_async(stream)
            .await
            .expect("Error during WebSocket handshake");
        println!("New WebSocket connection established");

        let (data_tx, data_rx) = tokio::sync::mpsc::channel(32);

        // Spawn a task to read data
        if let Some(file) = &args.file {
            tokio::spawn(read_attitude_from_file(file.clone(), data_tx));
        } else if let Some(_) = &args.serial {
            tokio::spawn(read_attitude_from_serial(args.clone(), data_tx));
        } else {
            panic!("Expected either a serial connection or input file");
        }

        // Spawn another task to compute estimates and send out via websocket
        tokio::spawn(run_estimators(args.clone(), data_rx, ws_stream));
    }
}

#[tokio::main]
async fn main() {
    let args = CliArgs::parse();
    run_server(args).await;
}
