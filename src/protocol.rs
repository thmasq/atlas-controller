#![allow(clippy::cast_possible_truncation, clippy::cast_possible_wrap)]

const FRAME_START: u8 = 0xAA;
const MAX_PAYLOAD: u8 = 8;

/// Commands received from the UART.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    Drive { left: i8, right: i8 },
    Stop,
    EmergencyStop,
    SensorPoll,
    Lift { power: i8 },
}

/// Errors that can occur during frame parsing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    InvalidLength,
    InvalidCrc,
    UnknownCommand(u8),
}

/// Responses to send back over UART.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Response {
    SensorStatus { flags: u8, heading_deg: f32 },
}

impl Response {
    /// Serializes the response into the provided buffer.
    /// Returns the number of bytes written, or `None` if the buffer is too small.
    pub fn build_frame(&self, out: &mut [u8]) -> Option<usize> {
        match self {
            Self::SensorStatus { flags, heading_deg } => {
                const LEN_SENSOR_STATUS: usize = 5;
                const TOTAL_FRAME_SIZE: usize = 1 + 1 + 1 + LEN_SENSOR_STATUS + 1;

                if out.len() < TOTAL_FRAME_SIZE {
                    return None;
                }

                out[0] = FRAME_START;
                out[1] = 0x10;
                out[2] = LEN_SENSOR_STATUS as u8;
                out[3] = *flags;
                out[4..8].copy_from_slice(&heading_deg.to_le_bytes());

                let crc = crc8_maxim(&out[1..8]);
                out[8] = crc;

                Some(TOTAL_FRAME_SIZE)
            }
        }
    }
}

/// Internal state for the `FrameParser`.
#[derive(Debug, Clone, Copy, Default)]
enum ParserState {
    #[default]
    WaitStart,
    Cmd,
    Len {
        cmd: u8,
    },
    Payload {
        cmd: u8,
        len: u8,
        buffer: [u8; MAX_PAYLOAD as usize],
        idx: usize,
    },
    Crc {
        cmd: u8,
        len: u8,
        buffer: [u8; MAX_PAYLOAD as usize],
    },
}

/// Parses incoming UART bytes into `Command`s.
#[derive(Default)]
pub struct FrameParser {
    state: ParserState,
}

impl FrameParser {
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Resets the parser to its initial state.
    pub const fn reset(&mut self) {
        self.state = ParserState::WaitStart;
    }

    /// Pushes a single byte into the parser.
    /// Returns `Some(Ok(Command))` if a full, valid frame was completed.
    /// Returns `Some(Err(ParseError))` if an invalid frame was detected.
    /// Returns `None` if the frame is still in progress.
    pub fn push(&mut self, byte: u8) -> Option<Result<Command, ParseError>> {
        match self.state {
            ParserState::WaitStart => {
                if byte == FRAME_START {
                    self.state = ParserState::Cmd;
                }
                None
            }
            ParserState::Cmd => {
                self.state = ParserState::Len { cmd: byte };
                None
            }
            ParserState::Len { cmd } => {
                let len = byte;
                if len > MAX_PAYLOAD {
                    self.reset();
                    return Some(Err(ParseError::InvalidLength));
                }
                if len == 0 {
                    self.state = ParserState::Crc {
                        cmd,
                        len,
                        buffer: [0; MAX_PAYLOAD as usize],
                    };
                } else {
                    self.state = ParserState::Payload {
                        cmd,
                        len,
                        buffer: [0; MAX_PAYLOAD as usize],
                        idx: 0,
                    };
                }
                None
            }
            ParserState::Payload {
                cmd,
                len,
                mut buffer,
                mut idx,
            } => {
                buffer[idx] = byte;
                idx += 1;
                if idx == len as usize {
                    self.state = ParserState::Crc { cmd, len, buffer };
                } else {
                    self.state = ParserState::Payload {
                        cmd,
                        len,
                        buffer,
                        idx,
                    };
                }
                None
            }
            ParserState::Crc { cmd, len, buffer } => {
                self.reset();

                let mut crc_buf = [0u8; 2 + MAX_PAYLOAD as usize];
                crc_buf[0] = cmd;
                crc_buf[1] = len;
                let payload_len = len as usize;
                crc_buf[2..2 + payload_len].copy_from_slice(&buffer[..payload_len]);

                let expected_crc = crc8_maxim(&crc_buf[..2 + payload_len]);

                if byte != expected_crc {
                    return Some(Err(ParseError::InvalidCrc));
                }

                let cmd_result = match cmd {
                    0x01 if len == 2 => Ok(Command::Drive {
                        left: buffer[0] as i8,
                        right: buffer[1] as i8,
                    }),
                    0x02 if len == 0 => Ok(Command::Stop),
                    0x03 if len == 0 => Ok(Command::EmergencyStop),
                    0x04 if len == 0 => Ok(Command::SensorPoll),
                    0x05 if len == 1 => Ok(Command::Lift {
                        power: buffer[0] as i8,
                    }),

                    0x01..=0x05 => Err(ParseError::InvalidLength),

                    _ => Err(ParseError::UnknownCommand(cmd)),
                };

                Some(cmd_result)
            }
        }
    }
}

/// Calculates the CRC-8/MAXIM checksum.
/// Polynomial 0x31, init 0x00, reflect in/out.
#[must_use]
pub fn crc8_maxim(data: &[u8]) -> u8 {
    let mut crc = 0x00u8;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            if (crc & 0x01) != 0 {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_crc8_maxim() {
        let data = [0x01, 0x02, 0x50, 0x50];
        let crc = crc8_maxim(&data);
        assert_eq!(crc, crc8_maxim(&data));
    }

    #[test]
    fn test_parse_drive_frame() {
        let mut parser = FrameParser::new();

        let cmd = 0x01;
        let len = 2;
        let payload = [50u8, 200u8];

        let mut crc_buf = [cmd, len, payload[0], payload[1]];
        let crc = crc8_maxim(&crc_buf);

        let frame = [FRAME_START, cmd, len, payload[0], payload[1], crc];

        let mut result = None;
        for &b in &frame {
            result = parser.push(b);
        }

        assert_eq!(
            result,
            Some(Ok(Command::Drive {
                left: 50,
                right: -56
            }))
        );
    }

    #[test]
    fn test_parse_stop_frame() {
        let mut parser = FrameParser::new();

        let crc = crc8_maxim(&[0x02, 0]);
        let frame = [FRAME_START, 0x02, 0, crc];

        let mut result = None;
        for &b in &frame {
            result = parser.push(b);
        }

        assert_eq!(result, Some(Ok(Command::Stop)));
    }

    #[test]
    fn test_build_sensor_status() {
        let resp = Response::SensorStatus {
            flags: 0b01010101,
            heading_deg: 90.5,
        };
        let mut buf = [0u8; 16];

        let written = resp.build_frame(&mut buf).unwrap();
        assert_eq!(written, 9);

        assert_eq!(buf[0], FRAME_START);
        assert_eq!(buf[1], 0x10);
        assert_eq!(buf[2], 5);
        assert_eq!(buf[3], 0b01010101);

        let heading_bytes = 90.5f32.to_le_bytes();
        assert_eq!(&buf[4..8], &heading_bytes);

        let expected_crc = crc8_maxim(&buf[1..8]);
        assert_eq!(buf[8], expected_crc);
    }
}
