
use hifive1::sprint;

const LINE_LEN: usize = 128;
const MAX_CURSOR: usize = LINE_LEN - 1;
const HISTORY_LEN: usize = 16;
const MAX_HISTORY: usize = HISTORY_LEN - 1;
pub struct Console {
    buffer: [[u8; LINE_LEN]; HISTORY_LEN],
    line : usize,
    cursor: usize,
    mode: Mode,
}

/// State machine for parsing state
enum Mode {
    None,
    SeenArrowFirst,
    SeenArrowSecond,
}
enum Arrow {
    Up,
    Down,
    Left,
    Right,
}

fn get_arrow(c: u8) -> Result<Arrow, ()> {
    match c {
        0x41 => Ok(Arrow::Up),
        0x42 => Ok(Arrow::Down),
        0x43 => Ok(Arrow::Right),
        0x44 => Ok(Arrow::Left),
        _ => Err(()),
    }
}

impl Console {
    pub fn new() -> Self {
        Console {
            buffer: [[0; LINE_LEN]; HISTORY_LEN],
            line: 0,
            cursor: 0,
            mode: Mode::None,
        }
    }

    /// Helper to increment line index without modulo.
    fn inc_cursor_idx(&mut self) {
        match self.cursor {
            MAX_CURSOR => self.write_newline(),
            _ => self.cursor += 1,
        }
    }

    /// Move cursor one back, won't move past 0.
    fn dec_cursor_idx(&mut self) {
        match self.cursor {
            0 => {},
            _ => self.cursor -= 1, 
        }
    }

    // Move down a line
    fn inc_line_idx(&mut self) {
        match self.line {
            MAX_HISTORY => self.line = 0,
            _ => self.line += 1,
        }
    }

    // Move up a line
    fn dec_line_idx(&mut self) {
        match self.line {
            0 => self.line = MAX_HISTORY,
            _ => self.line -= 1,
        }
    }

    /// Go to next line in history, clear that line out, set cursor.
    fn next_line(&mut self) {
        self.inc_line_idx();
        for elem in self.buffer[self.line].iter_mut() {
            *elem = 0;
        }
        self.cursor = 0;
    }

    /// Write character into console and history
    fn write_character(&mut self, c: char) {
        sprint!("{}", c);
        self.buffer[self.line][self.cursor] =  c as u8;
        self.inc_cursor_idx();
    }

    /// Write a backspace character 
    fn write_backspace(&mut self) {
        let backspace = 8 as char;
        sprint!{"{0} {0}", backspace}
        self.dec_cursor_idx();
        self.buffer[self.line][self.cursor] = 0;
    }

    fn write_newline(&mut self) {
        let newline = 10 as char;
        sprint!{"{}", newline}
        self.next_line();
    }

    fn clear_console_line(&mut self) {
        sprint!{"\r"};
        for _ in 0..LINE_LEN {
            sprint!{" "};
        }
        sprint!{"\r"};
    }

    /// Writes entire line to console, setting cursor to next character to input
    /// into. 
    fn write_line(&mut self) {
        self.cursor = 0;
        let iter  =self.buffer[self.line]
            .iter()
            .take_while(|c| **c != 0);

        for c in iter {
            sprint!{"{}", *c as char};
            self.cursor += 1;
        }
    }

    fn do_arrow_up(&mut self) {
        self.clear_console_line();
        self.dec_line_idx();
        self.write_line();
    }

    fn do_arrow_down(&mut self) {
        self.clear_console_line();
        self.inc_line_idx();
        self.write_line();
    }
    
    fn handle_arrow(&mut self, c: u8) {
        match get_arrow(c) {
            Ok(Arrow::Up) => self.do_arrow_up(),
            Ok(Arrow::Down) => self.do_arrow_down(),
            Ok(Arrow::Right) => {
                self.write_character('=');
                self.write_character('>');
            },
            Ok(Arrow::Left) => {
                self.write_character('<');
                self.write_character('=');
            },
            Err(_) => {},
        }
    }

    pub fn handle_character(&mut self, c: u8) {
        match (&self.mode, c) {
            // Catch arrow sequence 0x1b,0x5b,<0x41..=0x44>
            (Mode::None, 0x1b) => self.mode = Mode::SeenArrowFirst,
            (Mode::SeenArrowFirst, 0x5b) => self.mode = Mode::SeenArrowSecond,
            (Mode::SeenArrowSecond, 0x41..=0x44) => {
                self.handle_arrow(c);
                self.mode = Mode::None;
            },

            // Not Arrow, default to writing character
            (Mode::SeenArrowFirst , _) | 
            (Mode::SeenArrowSecond, _) => {
                self.write_character(c as char);
                self.mode = Mode::None;
            },

            // Not Arrow mode
            (Mode::None, 127) => self.write_backspace(),
            (Mode::None, 13) => self.write_newline(),
            (_, _) => self.write_character(c as char),
        }
    }
}
