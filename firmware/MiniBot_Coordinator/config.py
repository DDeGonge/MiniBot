# =============================================================================
# config.py  —  MiniBot Chess Swarm Coordinator
# All constants and "magic numbers" live here, broken out by section.
# =============================================================================
import math

# ---------------------------------------------------------------------------
# BOARD — physical geometry (all units in mm unless noted)
# ---------------------------------------------------------------------------
class BOARD:
    SQUARE_SIZE_MM         = 50       # one chess square side length
    NUM_SQUARES            = 8        # 8×8 board
    PLAYING_AREA_MM        = SQUARE_SIZE_MM * NUM_SQUARES  # 400 mm

    BORDER_TOP_MM          = 10
    BORDER_BOTTOM_MM       = 10
    BORDER_LEFT_MM         = 100
    BORDER_RIGHT_MM        = 100

    # Total canvas in mm (logical coordinate space)
    CANVAS_WIDTH_MM        = PLAYING_AREA_MM + BORDER_LEFT_MM + BORDER_RIGHT_MM  # 650
    CANVAS_HEIGHT_MM       = PLAYING_AREA_MM + BORDER_TOP_MM  + BORDER_BOTTOM_MM  # 450

    # Coordinate origin: (0,0) = bottom-left corner of the playing area
    # X increases → right, Y increases → up (standard math orientation)

    OUTLINE_THICKNESS_MM   = 2.0      # hard outer border line width
    GRID_LINE_THICKNESS_MM = 0.5


# ---------------------------------------------------------------------------
# PIECES — IDs, geometry, home positions
# ---------------------------------------------------------------------------
class PIECES:
    # ID ranges
    WHITE_ID_START         = 0x01
    WHITE_ID_END           = 0x11     # 17 white pieces (0x01–0x11)
    BLACK_ID_START         = 0x12
    BLACK_ID_END           = 0x22     # 17 black pieces (0x12–0x22)
    TOTAL_PIECES           = 34

    # Piece rank characters used for labels
    RANK_CHAR = {
        'king':   'K',
        'queen':  'Q',
        'rook':   'R',
        'bishop': 'B',
        'knight': 'N',
        'pawn':   'P',
    }

    CIRCLE_RADIUS_MM       = 15.5       # piece circle radius
    ORIENTATION_LINE_MM    = 16       # length of orientation marker from center
    ORIENTATION_LINE_WIDTH_MM = 1.5

    LABEL_FONT_SIZE_PT     = 9
    ID_FONT_SIZE_PT        = 6

    # Home positions: maps piece ID (int) → (x_mm, y_mm, theta_deg)
    # x,y are center of piece; theta=0 means facing +X (to the right)
    # Origin (0,0) is bottom-left corner of the PLAYING AREA.
    # Squares are labeled a-h (columns, x) and 1-8 (rows, y).
    # Square center col c, row r  →  x = (c-0.5)*SQUARE_SIZE, y = (r-0.5)*SQUARE_SIZE
    _S = 50  # shorthand for SQUARE_SIZE_MM

    # White pieces (IDs 0x01–0x11)
    # Standard back rank: R  N  B  Q  K  B  N  R  (cols 1–8)
    # Extra queen staged in left border (off-board, x < 0)
    HOME_POSITIONS = {
        # White pawns  (IDs 0x01–0x08, row 2)
        0x01: ( 1*_S - _S//2,  2*_S - _S//2,   90),
        0x02: ( 2*_S - _S//2,  2*_S - _S//2,   90),
        0x03: ( 3*_S - _S//2,  2*_S - _S//2,   90),
        0x04: ( 4*_S - _S//2,  2*_S - _S//2,   90),
        0x05: ( 5*_S - _S//2,  2*_S - _S//2,   90),
        0x06: ( 6*_S - _S//2,  2*_S - _S//2,   90),
        0x07: ( 7*_S - _S//2,  2*_S - _S//2,   90),
        0x08: ( 8*_S - _S//2,  2*_S - _S//2,   90),
        # White back rank  (IDs 0x09–0x10)
        0x09: ( 1*_S - _S//2,  1*_S - _S//2,   90),  # Rook a1
        0x0A: ( 2*_S - _S//2,  1*_S - _S//2,   90),  # Knight b1
        0x0B: ( 3*_S - _S//2,  1*_S - _S//2,   90),  # Bishop c1
        0x0C: ( 4*_S - _S//2,  1*_S - _S//2,   90),  # Queen d1
        0x0D: ( 5*_S - _S//2,  1*_S - _S//2,   90),  # King e1
        0x0E: ( 6*_S - _S//2,  1*_S - _S//2,   90),  # Bishop f1
        0x0F: ( 7*_S - _S//2,  1*_S - _S//2,   90),  # Knight g1
        0x10: ( 8*_S - _S//2,  1*_S - _S//2,   90),  # Rook h1
        # White extra queen — staged in left border (off-board)
        0x11: (9*_S - _S//2,  1*_S - _S//2,   90),

        # Black pawns  (IDs 0x12–0x19, row 7)
        0x12: ( 1*_S - _S//2,  7*_S - _S//2, 270),
        0x13: ( 2*_S - _S//2,  7*_S - _S//2, 270),
        0x14: ( 3*_S - _S//2,  7*_S - _S//2, 270),
        0x15: ( 4*_S - _S//2,  7*_S - _S//2, 270),
        0x16: ( 5*_S - _S//2,  7*_S - _S//2, 270),
        0x17: ( 6*_S - _S//2,  7*_S - _S//2, 270),
        0x18: ( 7*_S - _S//2,  7*_S - _S//2, 270),
        0x19: ( 8*_S - _S//2,  7*_S - _S//2, 270),
        # Black back rank  (IDs 0x1A–0x21)
        0x1A: ( 1*_S - _S//2,  8*_S - _S//2, 270),  # Rook a8
        0x1B: ( 2*_S - _S//2,  8*_S - _S//2, 270),  # Knight b8
        0x1C: ( 3*_S - _S//2,  8*_S - _S//2, 270),  # Bishop c8
        0x1D: ( 4*_S - _S//2,  8*_S - _S//2, 270),  # Queen d8
        0x1E: ( 5*_S - _S//2,  8*_S - _S//2, 270),  # King e8
        0x1F: ( 6*_S - _S//2,  8*_S - _S//2, 270),  # Bishop f8
        0x20: ( 7*_S - _S//2,  8*_S - _S//2, 270),  # Knight g8
        0x21: ( 8*_S - _S//2,  8*_S - _S//2, 270),  # Rook h8
        # Black extra queen — staged in left border (off-board)
        0x22: ( 0*_S - _S//2,  8*_S - _S//2, 270),
    }

    GRAVEYARD_POSITIONS = {
        # White pawns  (IDs 0x01–0x08, row 2)
        0x01: ( 10*_S - _S//2,  1*_S - _S//2,   90),
        0x02: ( 9.5*_S - _S//2,  1.5*_S - _S//2,   90),
        0x03: ( 10*_S - _S//2,  2*_S - _S//2,   90),
        0x04: ( 9.5*_S - _S//2,  2.5*_S - _S//2,   90),
        0x05: ( 10*_S - _S//2,  3*_S - _S//2,   90),
        0x06: ( 9.5*_S - _S//2,  3.5*_S - _S//2,   90),
        0x07: ( 10*_S - _S//2,  4*_S - _S//2,   90),
        0x08: ( 9.5*_S - _S//2,  4.5*_S - _S//2,   90),
        # White back rank  (IDs 0x09–0x10)
        0x09: ( 10*_S - _S//2,  5*_S - _S//2,   90),  # Rook a1
        0x0A: ( 9.5*_S - _S//2,  5.5*_S - _S//2,   90),  # Knight b1
        0x0B: ( 10*_S - _S//2,  6*_S - _S//2,   90),  # Bishop c1
        0x0C: ( 9.5*_S - _S//2,  6.5*_S - _S//2,   90),  # Queen d1
        0x0D: ( 10*_S - _S//2,  7*_S - _S//2,   90),  # King e1
        0x0E: ( 9.5*_S - _S//2,  7.5*_S - _S//2,   90),  # Bishop f1
        0x0F: ( 10*_S - _S//2,  8*_S - _S//2,   90),  # Knight g1
        0x10: ( 9*_S - _S//2,  8*_S - _S//2,   90),  # Rook h1
        # White extra queen — staged in left border (off-board)
        0x11: (9*_S - _S//2,  1*_S - _S//2,   90),

        # Black pawns  (IDs 0x12–0x19, row 7)
        0x12: ( -1*_S - _S//2,  1*_S - _S//2, 270),
        0x13: ( -.5*_S - _S//2,  1.5*_S - _S//2, 270),
        0x14: ( -1*_S - _S//2,  2*_S - _S//2, 270),
        0x15: ( -.5*_S - _S//2,  2.5*_S - _S//2, 270),
        0x16: ( -1*_S - _S//2,  3*_S - _S//2, 270),
        0x17: ( -.5*_S - _S//2,  3.5*_S - _S//2, 270),
        0x18: ( -1*_S - _S//2,  4*_S - _S//2, 270),
        0x19: ( -.5*_S - _S//2,  4.5*_S - _S//2, 270),
        # Black back rank  (IDs 0x1A–0x21)
        0x1A: ( -1*_S - _S//2,  5*_S - _S//2, 270),  # Rook a8
        0x1B: ( -.5*_S - _S//2,  5.5*_S - _S//2, 270),  # Knight b8
        0x1C: ( -1*_S - _S//2,  6*_S - _S//2, 270),  # Bishop c8
        0x1D: ( -.5*_S - _S//2,  6.5*_S - _S//2, 270),  # Queen d8
        0x1E: ( -1*_S - _S//2,  7*_S - _S//2, 270),  # King e8
        0x1F: ( -.5*_S - _S//2,  7.5*_S - _S//2, 270),  # Bishop f8
        0x20: ( -1*_S - _S//2,  8*_S - _S//2, 270),  # Knight g8
        0x21: ( 0*_S - _S//2,  1*_S - _S//2, 270),  # Rook h8
        # Black extra queen — staged in left border (off-board)
        0x22: ( 0*_S - _S//2,  8*_S - _S//2, 270),
    }

    # Rank assignments per piece ID
    PIECE_RANKS = {
        # White
        0x01: 'pawn', 0x02: 'pawn', 0x03: 'pawn', 0x04: 'pawn',
        0x05: 'pawn', 0x06: 'pawn', 0x07: 'pawn', 0x08: 'pawn',
        0x09: 'rook',   0x0A: 'knight', 0x0B: 'bishop', 0x0C: 'queen',
        0x0D: 'king',   0x0E: 'bishop', 0x0F: 'knight', 0x10: 'rook',
        0x11: 'queen',  # extra queen
        # Black
        0x12: 'pawn', 0x13: 'pawn', 0x14: 'pawn', 0x15: 'pawn',
        0x16: 'pawn', 0x17: 'pawn', 0x18: 'pawn', 0x19: 'pawn',
        0x1A: 'rook',   0x1B: 'knight', 0x1C: 'bishop', 0x1D: 'queen',
        0x1E: 'king',   0x1F: 'bishop', 0x20: 'knight', 0x21: 'rook',
        0x22: 'queen',  # extra queen
    }

    # Color assignment per ID range
    WHITE_IDS = set(range(0x01, 0x12))
    BLACK_IDS = set(range(0x12, 0x23))


# ---------------------------------------------------------------------------
# COMM — serial / USB protocol constants
# ---------------------------------------------------------------------------
class COMM:
    DEFAULT_BAUD_RATE        = 921600
    DEFAULT_POLL_INTERVAL_MS = 2000   # milliseconds between auto-polls
    ENCODING                 = 'ascii'
    TERMINATOR               = '\n'
    MSG_PREFIX               = '>'    # all frames begin with this character

    # Command IDs (host → ESP32)
    CMD_MOTOR_TEST           = 0      # >0,{id},{mode},{duty1},{duty2}
    CMD_POSITION             = 1      # >1,{id},{x_mm},{y_mm},{theta_rad},{duration_ms}
    CMD_POSITION_REQUEST     = 2      # >2,{id}
    CMD_MAG_FIELD_REQUEST    = 5      # >5,{id}
    CMD_SYNC                 = 7      # >7
    CMD_ELECTROMAGNET        = 254    # >254,{0|1}
    CMD_PING                 = 255    # >255

    # Response IDs (ESP32 → host)
    RESP_ACK                 = 3      # >3,{id},{x_mm},{y_mm},{theta_rad},{timestamp_ms},{battery_v}
    RESP_NACK                = 4      # >4,{id},{err_type},{timestamp_ms}
    RESP_MAG_FIELD           = 6      # >6,{id},{bx},{by},{bz},{timestamp_ms}
    RESP_PONG                = 255    # >255
    RESP_PARSE_ERROR         = 'ERR'  # >ERR,{message}

    # Electromagnet state
    MAG_OFF                  = 0
    MAG_ON                   = 1

    DELIMITER                = ','

    # Serial read timeout (seconds) — controls how often worker checks stop flag
    READ_TIMEOUT_S           = 0.05

    # If the outgoing send queue exceeds this many frames, drop the oldest
    # ones and flush the receive buffer to avoid processing stale responses.
    SEND_QUEUE_MAX_DEPTH     = 4
    # If the receive buffer holds more than this many bytes, discard it.
    RX_FLUSH_THRESHOLD_BYTES = 512


# ---------------------------------------------------------------------------
# GUI — colors, fonts, window dimensions
# ---------------------------------------------------------------------------
class GUI:
    WINDOW_TITLE           = 'MiniBot Chess Swarm Coordinator'
    WINDOW_MIN_WIDTH       = 1300
    WINDOW_MIN_HEIGHT      = 750

    SCALE_FACTOR           = 1.5     # px per mm (canvas logical → screen pixels)

    # Chessboard square colors
    LIGHT_SQUARE_COLOR     = '#F0D9B5'
    DARK_SQUARE_COLOR      = '#B58863'

    # Border / background
    BOARD_BACKGROUND_COLOR = '#2C2C2C'
    BOARD_OUTLINE_COLOR    = '#111111'

    # Piece colors
    WHITE_PIECE_FILL       = '#EFEFEF'
    WHITE_PIECE_OUTLINE    = '#222222'
    BLACK_PIECE_FILL       = '#333333'
    BLACK_PIECE_OUTLINE    = '#EFEFEF'

    # Selection / target indicators
    SELECTED_HIGHLIGHT_COLOR   = '#FFD700'   # gold
    TARGET_INDICATOR_COLOR     = '#00BFFF'   # deep sky blue
    ORIENTATION_LINE_COLOR_WHITE = '#222222'
    ORIENTATION_LINE_COLOR_BLACK = '#EFEFEF'

    # Staging area label color
    STAGED_PIECE_OPACITY   = 180             # 0–255 alpha

    # Control panel
    CONTROL_PANEL_MIN_WIDTH = 550

    # Position tracker table columns
    TRACKER_COLUMNS        = ['ID', 'Color', 'Rank', 'X (mm)', 'Y (mm)', 'θ (°)', 'Batt (V)', 'Last Update']


# ---------------------------------------------------------------------------
# PLANNING — path planner registry
# ---------------------------------------------------------------------------
class PLANNING:
    # Each entry: display name → dotted module path: class name
    PLANNERS = {
        'Enhanced Conflict':  ('planning.enhanced_conflict_planner', 'EnhancedConflictPlanner'),
        'Direct (debug only)':('planning.direct_planner',            'DirectPlanner'),
        
    }

    DEFAULT_PLANNER        = 'Enhanced Conflict'

    # Move duration defaults
    DEFAULT_MOVE_DURATION_MS = 3000
    HOME_MOVE_DURATION_MS    = 5000

    # SwarmPlanner tuning (retired — kept for reference)
    # SWARM_GRID_MM, SWARM_MAX_ITER, SWARM_MAX_STALLS removed

    # EnhancedConflictPlanner / ConflictPlanner tuning
    CONFLICT_MIN_SEGMENT_MM       = 20.0
    CONFLICT_ARRIVAL_EPS_MM       = 2.0
    CONFLICT_DOCK_EPS_MM          = 8.0
    CONFLICT_MAX_ITERATIONS       = 200
    CONFLICT_MAX_SEGMENT_MM       = 200.0   # longest single wave segment allowed
    CONFLICT_MIN_SEGMENT_MW_MM    = 5.0     # min segment for make-way routines
    CONFLICT_DETOUR_DISTANCE_MM   = 100.0
    CONFLICT_DETOUR_HOLD_ITERS    = 3
    CONFLICT_MAX_DETOUR_CHAIN     = 10
    CONFLICT_SERIAL_WAVE_SLACK_MS = 300
    CONFLICT_DEBUG_LOGGING        = True
    CONFLICT_ENABLE_UNSTICK       = True
    CONFLICT_UNSTICK_GAP_MM       = 5.0
    CONFLICT_UNSTICK_MAX_WAVES    = 10

    # EnhancedConflictPlanner tuning
    # Stall count at which make-way is attempted (before the hard _MAX_STALL cap).
    MAKE_WAY_DEADLOCK_TRIGGER  = 4
    # Maximum make-way interventions allowed per plan() call.
    MAKE_WAY_MAX_CYCLES        = 5
    # Maximum clearing rounds per make-way cycle (handles cascaded blockers).
    MAKE_WAY_PARK_ROUNDS       = 8
    # Maximum times the priority piece may yield (step aside) per make-way call.
    MAKE_WAY_YIELD_MAX         = 2
    # Net-progress stall: if the closest remaining piece doesn't close by this
    # many mm over NET_STALL_CAP consecutive iterations, force an exit.
    MAKE_WAY_NET_PROGRESS_MM   = 5.0
    MAKE_WAY_NET_STALL_CAP     = 14
    # Cycle detector: fingerprint grid resolution (mm) and history depth.
    MAKE_WAY_CYCLE_GRID_MM     = 5
    MAKE_WAY_CYCLE_HISTORY     = 10

    # "Optimize Assignment" exhaustive search cap: max planner calls per run.
    OPTIMIZE_ASSIGNMENT_MAX_TRIALS  = 250
    # Number of parallel worker processes used by the exhaustive search.
    OPTIMIZE_ASSIGNMENT_WORKERS     = 16

    # StagingPlanner — pawn staging squares (centre positions in mm)
    # White left cluster: a2, a3, b2, b3
    STAGING_WHITE_LEFT  = [(25, 75), (25, 125), (75, 75), (75, 125)]
    # White right cluster: g2, g3, h2, h3
    STAGING_WHITE_RIGHT = [(325, 75), (325, 125), (375, 75), (375, 125)]
    # Black left cluster: a6, a7, b6, b7  (mirrors white left)
    STAGING_BLACK_LEFT  = [(25, 275), (25, 325), (75, 275), (75, 325)]
    # Black right cluster: g6, g7, h6, h7  (mirrors white right)
    STAGING_BLACK_RIGHT = [(325, 275), (325, 325), (375, 275), (375, 325)]
    # Y coordinate of the outer staging tier that requires a phase-2 move
    STAGING_OUTER_Y_WHITE = 125   # rank 3  (white outer tier → move down to rank 2)
    STAGING_OUTER_Y_BLACK = 275   # rank 6  (black outer tier → move up to rank 7)


# ---------------------------------------------------------------------------
# CHESS — rules engine integration stubs
# ---------------------------------------------------------------------------
class CHESS:
    # Default starting FEN
    STARTING_FEN = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1'

    # When a chess rules engine is integrated, set this to the engine adapter
    # callable: validate_move(fen: str, move_uci: str) -> bool
    # Leave as None to disable rules enforcement.
    RULES_ENGINE_ADAPTER   = None


# ---------------------------------------------------------------------------
# ELECTROMAGNETS — physical electromagnet geometry and positions
# All positions are in playing-area mm (same origin as piece coordinates).
# ---------------------------------------------------------------------------
class ELECTROMAGNETS:
    # Ring geometry (mm)
    OD_MM = 60.0   # outer diameter of the drawn ring indicator
    ID_MM = 20.0   # inner diameter of the drawn ring indicator

    # Centre positions (x_mm, y_mm) of each electromagnet in playing-area coords
    POSITIONS = [
        (250.0, 156.7),
        (150.0, 156.7),
        (200.0, 243.3),
    ]


# ---------------------------------------------------------------------------
# SIMULATOR — software-in-the-loop motion simulator
# ---------------------------------------------------------------------------
class SIMULATOR:
    # Nominal robot speed used when simulator is active
    DEFAULT_SPEED_MM_S     = 80.0

    # Timer tick rate for simulated motion (20 Hz)
    UPDATE_INTERVAL_MS     = 50

    # Rotation angular velocity in rad/s (used to compute move duration)
    ROTATION_ANGULAR_VEL_RAD_S = 2.0

    # Maximum rotation rate in degrees per second (derived)
    ROTATION_SPEED_DEG_S   = math.degrees(ROTATION_ANGULAR_VEL_RAD_S)

    # Heading must be within this many degrees of the target bearing before
    # the robot begins translating (differential drive constraint).
    HEADING_TOLERANCE_DEG  = 3.0

    # Extra clearance added to 2×radius for piece–piece collision detection.
    # Set to 0 so pieces only block when they actually touch (edge-to-edge).
    COLLISION_MARGIN_MM    = -3.0

    # Physical boundary of the whole table in playing-area mm coordinates.
    # Pieces must stay within these limits (centre must be ≥ radius from wall).
    # Derived from BOARD constants — edit BOARD values to change these.
    X_MIN_MM = -BOARD.BORDER_LEFT_MM                              # -125
    X_MAX_MM =  BOARD.PLAYING_AREA_MM + BOARD.BORDER_RIGHT_MM    #  525
    Y_MIN_MM = -BOARD.BORDER_BOTTOM_MM                            #  -25
    Y_MAX_MM =  BOARD.PLAYING_AREA_MM + BOARD.BORDER_TOP_MM      #  425
