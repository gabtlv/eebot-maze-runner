; FINAL PROJECT: EEBOT MAZE RUNNER
; *****************************************************************
; * Single-file absolute assembly program for the HCS12           *
; * Implements an autonomous maze-navigating robot (EEBOT)        *
; * using IR sensors, bump switches, PWM motor control,           *
; * an LCD display, and a timer overflow interrupt.               *
; *****************************************************************

              XDEF Entry, _Startup
              ABSENTRY Entry
              INCLUDE "derivative.inc"

;***************************************************************************************************
; EQUATES
;***************************************************************************************************

; --- LCD Constants ---
CLEAR_HOME    EQU   $01       ; Command: clear screen and move cursor to home position
INTERFACE     EQU   $38       ; Command: 8-bit bus width, 2-line display mode
CURSOR_OFF    EQU   $0C       ; Command: turn display on, hide the cursor
SHIFT_OFF     EQU   $06       ; Command: auto-increment address, no display shift
LCD_SEC_LINE  EQU   64        ; DDRAM address of the first character on line 2

; --- LCD Pin Mapping ---
LCD_CNTR      EQU   PTJ       ; Port J drives the LCD control lines (E = PJ7, RS = PJ6)
LCD_DAT       EQU   PORTB     ; Port B carries the 8-bit LCD data bus
LCD_E         EQU   $80       ; Bitmask for the LCD Enable (E) pin on PTJ
LCD_RS        EQU   $40       ; Bitmask for the LCD Register Select (RS) pin on PTJ

; --- Misc ASCII / String Constants ---
NULL          EQU   00        ; Null terminator used to end strings
CR            EQU   $0D       ; ASCII carriage return
SPACE         EQU   ' '       ; ASCII space character

; --- Turn Timing (TOF ticks) ---
T_LEFT        EQU   8         ; Duration of a left turn in TOF counter ticks
T_RIGHT       EQU   8         ; Duration of a right turn in TOF counter ticks

; --- Robot State Definitions ---
START         EQU   0         ; Waiting for initial button press to begin
FWD           EQU   1         ; Driving straight forward
ALL_STOP      EQU   2         ; All motors off, robot halted
LEFT_TRN      EQU   3         ; Executing a timed left turn
RIGHT_TRN     EQU   4         ; Executing a timed right turn
REV_TRN       EQU   5         ; Reversing then turning to avoid a front obstacle
LEFT_ALIGN    EQU   6         ; Fine-aligning to the left to stay on the line
RIGHT_ALIGN   EQU   7         ; Fine-aligning to the right to stay on the line

;***************************************************************************************************
; DATA SEGMENT  (HCS12 RAM: $3800 - $3FFF)
;***************************************************************************************************
              ORG   $3800

; --- Sensor Baseline Readings (measured at calibration) ---
BASE_LINE     FCB   $9D       ; Expected reading for the line sensor
BASE_BOW      FCB   $CA       ; Expected reading for the front (bow) sensor
BASE_MID      FCB   $CA       ; Expected reading for the center sensor
BASE_PORT     FCB   $CC       ; Expected reading for the left (port) sensor
BASE_STBD     FCB   $CC       ; Expected reading for the right (starboard) sensor

; --- Per-sensor Variance Thresholds (tuned during testing) ---
LINE_VARIANCE           FCB   $18   ; Acceptable deviation for line sensor
BOW_VARIANCE            FCB   $30   ; Acceptable deviation for bow sensor
PORT_VARIANCE           FCB   $20   ; Acceptable deviation for port sensor
MID_VARIANCE            FCB   $20   ; Acceptable deviation for mid sensor
STARBOARD_VARIANCE      FCB   $15   ; Acceptable deviation for starboard sensor

; --- LCD Display Buffers ---
TOP_LINE      RMB   20        ; 20-character buffer for LCD line 1
              FCB   NULL      ; Null terminator
BOT_LINE      RMB   20        ; 20-character buffer for LCD line 2
              FCB   NULL      ; Null terminator
CLEAR_LINE    FCC   '                  '  ; Blank string used to wipe a display line
              FCB   NULL      ; Null terminator

TEMP          RMB   1         ; General-purpose scratch byte

; --- Live Sensor Readings (updated each main loop iteration) ---
SENSOR_LINE   FCB   $01       ; IR line sensor (initialized to test pattern)
SENSOR_BOW    FCB   $23       ; Front (bow) IR sensor
SENSOR_PORT   FCB   $45       ; Left (port) IR sensor
SENSOR_MID    FCB   $67       ; Center IR sensor
SENSOR_STBD   FCB   $89       ; Right (starboard) IR sensor
SENSOR_NUM    RMB   1         ; Loop index for READ_SENSORS (0-4)

;***************************************************************************************************
              ORG   $3850

; --- Timer and State Variables ---
TOF_COUNTER   dc.b  0         ; Incremented by the TOF ISR at ~23 Hz
CRNT_STATE    dc.b  2         ; Active robot state (starts in ALL_STOP)
T_TURN        ds.b  1         ; TOF value at which the current turn should end

; --- BCD Conversion Output Digits ---
TEN_THOUS     ds.b  1         ; 10,000s digit (ASCII after BCD2ASC)
THOUSANDS     ds.b  1         ; 1,000s digit
HUNDREDS      ds.b  1         ; 100s digit
TENS          ds.b  1         ; 10s digit
UNITS         ds.b  1         ; 1s digit
NO_BLANK      ds.b  1         ; Leading-zero suppression flag (0 = still blanking)

HEX_TABLE     FCC   '0123456789ABCDEF'  ; Lookup table for nibble-to-ASCII conversion
BCD_SPARE     RMB   2         ; Overflow guard bytes for BCD buffer

;***************************************************************************************************
; CODE SEGMENT
;***************************************************************************************************
              ORG   $4000
Entry:
_Startup:
              LDS   #$4000           ; Point the stack pointer to the top of code RAM
              CLI                    ; Unmask interrupts globally
              JSR   INIT             ; Configure all I/O port directions
              JSR   openADC          ; Power up and configure the ATD module
              JSR   initLCD          ; Send initialisation commands to the LCD
              JSR   CLR_LCD_BUF      ; Pre-fill both LCD buffers with spaces
              BSET  DDRA,%00000011   ; PA1/PA0 as outputs: STBD_DIR and PORT_DIR
              BSET  DDRT,%00110000   ; PT5/PT4 as outputs: STBD_SPEED and PORT_SPEED
              JSR   initAD           ; Re-initialise ATD for sensor reads
              JSR   initLCD          ; Re-send LCD init (ensures clean state)
              JSR   clrLCD           ; Clear the LCD and home the cursor
              LDX   #msg1
              JSR   putsLCD          ; Print "Battery volt " on line 1
              LDAA  #$C0
              JSR   cmd2LCD          ; Move cursor to line 2
              LDX   #msg2
              JSR   putsLCD          ; Print "State" on line 2
              JSR   ENABLE_TOF       ; Start the timer overflow interrupt

; --- Main Loop ---
MAIN
              JSR   G_LEDS_ON        ; Power the IR emitters
              JSR   READ_SENSORS     ; Sample all 5 IR sensors via ATD
              JSR   G_LEDS_OFF       ; Cut power to IR emitters
              JSR   UPDT_DISPL       ; Refresh battery voltage and state on LCD
              LDAA  CRNT_STATE       ; Load current state into A
              JSR   DISPATCHER       ; Jump to the appropriate state handler
              BRA   MAIN             ; Loop forever

;***************************************************************************************************
; CONSTANT DATA
;***************************************************************************************************
msg1          dc.b  "Battery volt ",0
msg2          dc.b  "State",0
tab           dc.b  "start  ",0      ; State name strings indexed by state number
              dc.b  "fwd    ",0
              dc.b  "all_stp",0
              dc.b  "LeftTurn  ",0
              dc.b  "RightTurn  ",0
              dc.b  "RevTrn ",0
              dc.b  "LeftTimed ",0
              dc.b  "RTimed ",0

;***************************************************************************************************
; DISPATCHER — routes execution to the correct state handler
;***************************************************************************************************
DISPATCHER        JSR   VERIFY_START
                  RTS

; Each VERIFY_x routine checks whether CRNT_STATE matches its state.
; If yes, it calls the handler and returns. If no, it falls through to the next check.

VERIFY_START      CMPA  #START
                  BNE   VERIFY_FORWARD
                  JSR   START_ST
                  RTS

VERIFY_FORWARD    CMPA  #FWD
                  BNE   VERIFY_STOP
                  JSR   FWD_ST
                  RTS

VERIFY_REV_TRN    CMPA  #REV_TRN
                  BNE   VERIFY_LEFT_ALIGN
                  JSR   REV_TRN_ST
                  RTS

VERIFY_STOP       CMPA  #ALL_STOP
                  BNE   VERIFY_LEFT_TRN
                  JSR   ALL_STOP_ST
                  RTS

VERIFY_LEFT_TRN   CMPA  #LEFT_TRN
                  BNE   VERIFY_RIGHT_TRN
                  JSR   LEFT
                  RTS

VERIFY_LEFT_ALIGN CMPA  #LEFT_ALIGN
                  BNE   VERIFY_RIGHT_ALIGN
                  JSR   LEFT_ALIGN_DONE
                  RTS

VERIFY_RIGHT_TRN  CMPA  #RIGHT_TRN
                  BNE   VERIFY_REV_TRN
                  JSR   RIGHT

VERIFY_RIGHT_ALIGN CMPA #RIGHT_ALIGN
                  JSR   RIGHT_ALIGN_DONE
                  RTS                       ; Falls here for unrecognised states too

;***************************************************************************************************
; STATE HANDLERS
;***************************************************************************************************

; --- START: wait for the bow bumper to be released, then begin driving forward ---
START_ST          BRCLR   PORTAD0, %00000100, RELEASE   ; If bumper is pressed, stay put
                  JSR     INIT_FWD                       ; Bumper released — start moving
                  MOVB    #FWD, CRNT_STATE               ; Transition to FWD state

RELEASE           RTS

; --- FWD: drive forward, handle bumps, and monitor sensors for course correction ---
FWD_ST            BRSET   PORTAD0, $04, NO_FWD_BUMP         ; Skip if bow bumper is clear
                  MOVB    #REV_TRN, CRNT_STATE              ; Bow hit — switch to reverse-turn
                  JSR     UPDT_DISPL                        ; Update display immediately
                  JSR     INIT_REV                          ; Reverse both motors
                  LDY     #6000
                  JSR     del_50us                          ; Reverse for 300 ms
                  JSR     INIT_RIGHT                        ; Begin turning right
                  LDY     #6000
                  JSR     del_50us                          ; Turn for 300 ms
                  LBRA    EXIT

NO_FWD_BUMP       BRSET   PORTAD0, $04, NO_FWD_REAR_BUMP    ; Check stern bumper
                  MOVB    #ALL_STOP, CRNT_STATE             ; Stern hit — stop
                  JSR     INIT_STOP
                  LBRA    EXIT

; Sensor-based alignment logic while driving forward
NO_FWD_REAR_BUMP  LDAA    SENSOR_BOW
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     NOT_ALIGNED                       ; Bow sensor over threshold — off-path
                  LDAA    SENSOR_MID
                  ADDA    MID_VARIANCE
                  CMPA    BASE_MID
                  BPL     NOT_ALIGNED                       ; Mid sensor over threshold — off-path
                  LDAA    SENSOR_LINE
                  ADDA    LINE_VARIANCE
                  CMPA    BASE_LINE
                  BPL     CHECK_RIGHT_ALIGN                 ; Line sensor high — drift right
                  LDAA    SENSOR_LINE
                  SUBA    LINE_VARIANCE
                  CMPA    BASE_LINE
                  BMI     CHECK_LEFT_ALIGN                  ; Line sensor low — drift left

; Sensor deviation detected — decide turn direction
NOT_ALIGNED       LDAA    SENSOR_PORT
                  ADDA    PORT_VARIANCE
                  CMPA    BASE_PORT
                  BPL     PARTIAL_LEFT_TRN                  ; Port sensor sees obstacle — turn left
                  BMI     NO_PORT

NO_PORT           LDAA    SENSOR_BOW
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     EXIT                              ; Bow clear — nothing to do
                  BMI     NO_BOW

NO_BOW            LDAA    SENSOR_STBD
                  ADDA    STARBOARD_VARIANCE
                  CMPA    BASE_STBD
                  BPL     PARTIAL_RIGHT_TRN                 ; Starboard sees obstacle — turn right
                  BMI     EXIT

; Timed left partial turn
PARTIAL_LEFT_TRN  LDY     #6000
                  JSR     del_50us
                  JSR     INIT_LEFT
                  MOVB    #LEFT_TRN, CRNT_STATE
                  LDY     #6000
                  JSR     del_50us
                  BRA     EXIT

; Precise left alignment (line-follow correction)
CHECK_LEFT_ALIGN  JSR     INIT_LEFT
                  MOVB    #LEFT_ALIGN, CRNT_STATE
                  BRA     EXIT

; Timed right partial turn
PARTIAL_RIGHT_TRN LDY     #6000
                  JSR     del_50us
                  JSR     INIT_RIGHT
                  MOVB    #RIGHT_TRN, CRNT_STATE
                  LDY     #6000
                  JSR     del_50us
                  BRA     EXIT

; Precise right alignment (line-follow correction)
CHECK_RIGHT_ALIGN JSR     INIT_RIGHT
                  MOVB    #RIGHT_ALIGN, CRNT_STATE
                  BRA     EXIT

EXIT              RTS

; --- LEFT_TRN: keep turning left until the bow sensor clears ---
LEFT              LDAA    SENSOR_BOW
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     LEFT_ALIGN_DONE   ; Bow clear — alignment done
                  BMI     EXIT              ; Still blocked — keep turning

LEFT_ALIGN_DONE   MOVB    #FWD, CRNT_STATE  ; Path clear — resume forward
                  JSR     INIT_FWD
                  BRA     EXIT

; --- RIGHT_TRN: keep turning right until the bow sensor clears ---
RIGHT             LDAA    SENSOR_BOW
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BPL     RIGHT_ALIGN_DONE  ; Bow clear — alignment done
                  BMI     EXIT              ; Still blocked — keep turning

RIGHT_ALIGN_DONE  MOVB    #FWD, CRNT_STATE  ; Path clear — resume forward
                  JSR     INIT_FWD
                  BRA     EXIT

; --- REV_TRN: reverse-turning after a front collision ---
REV_TRN_ST        LDAA    SENSOR_BOW
                  ADDA    BOW_VARIANCE
                  CMPA    BASE_BOW
                  BMI     EXIT              ; Bow still blocked — wait
                  JSR     INIT_LEFT         ; Bow clear — pivot left
                  MOVB    #FWD, CRNT_STATE  ; Then resume forward
                  JSR     INIT_FWD
                  BRA     EXIT

; --- ALL_STOP: motors off, waiting for the bumper to be released ---
ALL_STOP_ST       BRSET   PORTAD0, %00000100, NO_START_BUMP  ; Bumper still pressed?
                  MOVB    #START, CRNT_STATE                 ; Released — go to START

NO_START_BUMP     RTS

;***************************************************************************************************
; MOTOR CONTROL
;***************************************************************************************************

; Set both motors to spin right (clockwise)
INIT_RIGHT        BSET    PORTA,%00000010   ; STBD_DIR = 1
                  BCLR    PORTA,%00000001   ; PORT_DIR = 0
                  LDAA    TOF_COUNTER
                  ADDA    #T_RIGHT
                  STAA    T_TURN            ; Record when the turn should end
                  RTS

; Set both motors to spin left (counter-clockwise)
INIT_LEFT         BSET    PORTA,%00000001   ; PORT_DIR = 1
                  BCLR    PORTA,%00000010   ; STBD_DIR = 0
                  LDAA    TOF_COUNTER
                  ADDA    #T_LEFT
                  STAA    T_TURN            ; Record when the turn should end
                  RTS

; Drive both motors forward
INIT_FWD          BCLR    PORTA, %00000011  ; Both direction bits low = forward
                  BSET    PTT, %00110000    ; Enable both motor PWM outputs
                  RTS

; Drive both motors in reverse
INIT_REV          BSET    PORTA,%00000011   ; Both direction bits high = reverse
                  BSET    PTT,%00110000     ; Enable both motor PWM outputs
                  RTS

; Cut power to both motors
INIT_STOP         BCLR    PTT, %00110000    ; Disable both motor PWM outputs
                  RTS

;***************************************************************************************************
; PORT INITIALISATION
;***************************************************************************************************
INIT              BCLR   DDRAD,$FF   ; PORTAD — all pins as inputs (for bump switches + ATD)
                  BSET   DDRA,$FF    ; PORTA  — all pins as outputs (motor dir + sensor select)
                  BSET   DDRB,$FF    ; PORTB  — all pins as outputs (LCD data bus)
                  BSET   DDRJ,$C0   ; PTJ bits 7-6 as outputs (LCD E and RS control lines)
                  RTS

;***************************************************************************************************
; ATD INITIALISATION (first-time power-up)
;***************************************************************************************************
openADC           MOVB   #$80,ATDCTL2   ; Enable the ATD module (bit 7 = ADPU)
                  LDY    #1
                  JSR    del_50us       ; Wait 50 us for ATD power-up stabilisation
                  MOVB   #$20,ATDCTL3   ; 4 conversions per sequence
                  MOVB   #$97,ATDCTL4   ; 8-bit resolution, ATD clock prescaler = 48
                  RTS

;***************************************************************************************************
; LCD BUFFER INITIALISATION
; Fills both TOP_LINE and BOT_LINE buffers with space characters.
; Call once at startup before building any display content.
;***************************************************************************************************
CLR_LCD_BUF       LDX   #CLEAR_LINE
                  LDY   #TOP_LINE
                  JSR   STRCPY

CLB_SECOND        LDX   #CLEAR_LINE
                  LDY   #BOT_LINE
                  JSR   STRCPY

CLB_EXIT          RTS

;***************************************************************************************************
; STRCPY — copy a null-terminated string
; In:  X = source address, Y = destination address
; Out: string copied byte-by-byte including the null terminator
;***************************************************************************************************
STRCPY            PSHX
                  PSHY
                  PSHA

STRCPY_LOOP       LDAA 0,X          ; Read next source byte
                  STAA 0,Y          ; Write to destination
                  BEQ STRCPY_EXIT   ; Stop when null terminator is copied
                  INX
                  INY
                  BRA STRCPY_LOOP

STRCPY_EXIT       PULA
                  PULY
                  PULX
                  RTS

;***************************************************************************************************
; G_LEDS_ON — power the IR guider emitters (PORTA bit 5 high)
;***************************************************************************************************
G_LEDS_ON         BSET PORTA,%00100000
                  RTS

;***************************************************************************************************
; G_LEDS_OFF — cut power to the IR guider emitters (PORTA bit 5 low)
;***************************************************************************************************
G_LEDS_OFF        BCLR PORTA,%00100000
                  RTS

;***************************************************************************************************
; READ_SENSORS — sample all 5 IR sensors and store results in SENSOR_x variables
; Cycles through sensor indices 0-4, selecting each via SELECT_SENSOR,
; waiting 10 ms for stabilisation, then performing a single ATD conversion.
;***************************************************************************************************
READ_SENSORS      CLR   SENSOR_NUM       ; Start with sensor index 0
                  LDX   #SENSOR_LINE     ; X points to first sensor storage byte

RS_MAIN_LOOP      LDAA  SENSOR_NUM
                  JSR   SELECT_SENSOR    ; Mux the chosen sensor onto the ATD input
                  LDY   #200
                  JSR   del_50us         ; 10 ms stabilisation delay
                  LDAA  #%10000001       ; Configure ATD: right-justify, single, ch AN1
                  STAA  ATDCTL5
                  BRCLR ATDSTAT0,$80,*   ; Spin until conversion complete flag set
                  LDAA  ATDDR0L          ; Read 8-bit result
                  STAA  0,X              ; Store in the appropriate sensor variable
                  CPX   #SENSOR_STBD
                  BEQ   RS_EXIT          ; All 5 sensors read — exit
                  INC   SENSOR_NUM
                  INX
                  BRA   RS_MAIN_LOOP

RS_EXIT           RTS

;***************************************************************************************************
; SELECT_SENSOR — route a sensor index (0-4) to PORTA bits 4:2
; In:  A = sensor number (0-4)
; Side effect: updates PORTA sensor-select bits, preserves other bits via TEMP
;***************************************************************************************************
SELECT_SENSOR     PSHA
                  LDAA PORTA
                  ANDA #%11100011       ; Mask out bits 4:2 (sensor select field)
                  STAA TEMP
                  PULA
                  ASLA                  ; Shift sensor index into bits 4:2 position
                  ASLA
                  ANDA #%00011100       ; Keep only the relevant bits
                  ORAA TEMP             ; Merge with preserved bits
                  STAA PORTA
                  RTS

;***************************************************************************************************
; DISPLAY_SENSORS — render all 5 sensor values as hex pairs on the LCD
;***************************************************************************************************
DP_FRONT_SENSOR   EQU TOP_LINE+3
DP_PORT_SENSOR    EQU BOT_LINE+0
DP_MID_SENSOR     EQU BOT_LINE+3
DP_STBD_SENSOR    EQU BOT_LINE+6
DP_LINE_SENSOR    EQU BOT_LINE+9

DISPLAY_SENSORS   LDAA  SENSOR_BOW
                  JSR   BIN2ASC
                  LDX   #DP_FRONT_SENSOR
                  STD   0,X

                  LDAA  SENSOR_PORT
                  JSR   BIN2ASC
                  LDX   #DP_PORT_SENSOR
                  STD   0,X

                  LDAA  SENSOR_MID
                  JSR   BIN2ASC
                  LDX   #DP_MID_SENSOR
                  STD   0,X

                  LDAA  SENSOR_STBD
                  JSR   BIN2ASC
                  LDX   #DP_STBD_SENSOR
                  STD   0,X

                  LDAA  SENSOR_LINE
                  JSR   BIN2ASC
                  LDX   #DP_LINE_SENSOR
                  STD   0,X

                  LDAA  #CLEAR_HOME
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us         ; Wait 2 ms for clear-display command to finish
                  LDX   #TOP_LINE
                  JSR   putsLCD
                  LDAA  #LCD_SEC_LINE
                  JSR   LCD_POS_CRSR
                  LDX   #BOT_LINE
                  JSR   putsLCD
                  RTS

;***************************************************************************************************
; UPDT_DISPL — update the LCD with current battery voltage and robot state name
; Battery voltage: raw ATD ch0 result scaled to millivolts via: V = result*39 + 600
; State name: looked up from the tab string table using CRNT_STATE * 8 as offset
;***************************************************************************************************
UPDT_DISPL        MOVB    #$90,ATDCTL5    ; Start ATD: right-just, unsigned, single, ch0
                  BRCLR   ATDSTAT0,$80,*  ; Wait for conversion to complete
                  LDAA    ATDDR0L         ; A = raw battery ADC result
                  LDAB    #39
                  MUL                     ; D = result * 39
                  ADDD    #600            ; D = result * 39 + 600  (millivolts)
                  JSR     int2BCD         ; Convert D to 5 BCD digits
                  JSR     BCD2ASC         ; Convert BCD digits to ASCII with zero-blanking
                  LDAA    #$8D            ; LCD cursor to end of "Battery volt " on line 1
                  JSR     cmd2LCD
                  LDAA    TEN_THOUS
                  JSR     putcLCD
                  LDAA    THOUSANDS
                  JSR     putcLCD
                  LDAA    #'.'
                  JSR     putcLCD         ; Insert decimal point (e.g. "12.34" V)
                  LDAA    HUNDREDS
                  JSR     putcLCD
                  LDAA    #$C7            ; LCD cursor to end of "State" on line 2
                  JSR     cmd2LCD
                  LDAB    CRNT_STATE
                  LSLB                    ; Multiply state by 8 (each label is 8 bytes)
                  LSLB
                  LSLB
                  LDX     #tab
                  ABX                     ; X now points to the correct state label
                  JSR     putsLCD
                  RTS

;***************************************************************************************************
; ENABLE_TOF — configure the timer module and enable the Timer Overflow Interrupt
; Prescaler = 16, so at 8 MHz E-clock: TOF fires at ~23 Hz
;***************************************************************************************************
ENABLE_TOF        LDAA    #%10000000
                  STAA    TSCR1           ; Enable the free-running TCNT counter
                  STAA    TFLG2           ; Clear any pending TOF flag
                  LDAA    #%10000100      ; TOI enable + prescale factor of 16
                  STAA    TSCR2
                  RTS

; TOF ISR — fires at ~23 Hz, increments TOF_COUNTER for turn timing
TOF_ISR           INC     TOF_COUNTER
                  LDAA    #%10000000
                  STAA    TFLG2           ; Acknowledge (clear) the TOF flag
                  RTI

;***************************************************************************************************
; LCD UTILITY SUBROUTINES
;***************************************************************************************************

; initLCD — send the 3-command initialisation sequence to the LCD controller
initLCD:          BSET    DDRB,%11111111  ; PORTB all outputs (LCD data bus)
                  BSET    DDRJ,%11000000  ; PTJ bits 7-6 as outputs (E, RS)
                  LDY     #2000
                  JSR     del_50us        ; Wait 100 ms for LCD power-up
                  LDAA    #$28
                  JSR     cmd2LCD         ; Function set: 4-bit mode, 2 lines
                  LDAA    #$0C
                  JSR     cmd2LCD         ; Display on, cursor off
                  LDAA    #$06
                  JSR     cmd2LCD         ; Entry mode: increment, no shift
                  RTS

; clrLCD — send the clear-display command and wait for it to complete
clrLCD:           LDAA  #$01
                  JSR   cmd2LCD
                  LDY   #40
                  JSR   del_50us          ; 2 ms required by HD44780 after clear
                  RTS

; del_50us — delay Y * 50 microseconds (inner loop = 300 NOPs @ 8 MHz ˜ 50 us)
del_50us          PSHX
eloop             LDX   #300
iloop             NOP
                  DBNE X,iloop
                  DBNE Y,eloop
                  PULX
                  RTS

; cmd2LCD — send a command byte (RS=0) to the LCD instruction register
cmd2LCD:          BCLR  LCD_CNTR, LCD_RS  ; RS low = instruction register
                  JSR   dataMov
                  RTS

; putsLCD — send a null-terminated string pointed to by X to the LCD
putsLCD:          LDAA  1,X+
                  BEQ   donePS
                  JSR   putcLCD
                  BRA   putsLCD
donePS            RTS

; putcLCD — send a single character byte (RS=1) to the LCD data register
putcLCD:          BSET  LCD_CNTR, LCD_RS  ; RS high = data register
                  JSR   dataMov
                  RTS

; dataMov — clock a byte in A to the LCD using the 4-bit interface (two nibbles)
dataMov:          BSET  LCD_CNTR, LCD_E   ; Raise E to begin write cycle
                  STAA  LCD_DAT           ; Put upper nibble on bus
                  BCLR  LCD_CNTR, LCD_E   ; Lower E to latch upper nibble
                  LSLA                    ; Shift lower nibble up into bits 7:4
                  LSLA
                  LSLA
                  LSLA
                  BSET  LCD_CNTR, LCD_E   ; Raise E for second write cycle
                  STAA  LCD_DAT           ; Put lower nibble on bus
                  BCLR  LCD_CNTR, LCD_E   ; Lower E to latch lower nibble
                  LDY   #1
                  JSR   del_50us          ; 50 us settling time for most LCD commands
                  RTS

; initAD — second ATD initialisation (sensor sampling configuration)
initAD            MOVB  #$C0,ATDCTL2      ; ATD on, fast flag clear mode
                  JSR   del_50us
                  MOVB  #$00,ATDCTL3      ; 8 conversions per sequence
                  MOVB  #$85,ATDCTL4      ; 8-bit resolution, 2 conversion clocks, prescaler=12
                  BSET  ATDDIEN,$0C       ; AN3 and AN2 configured as digital inputs (bump switches)
                  RTS

;***************************************************************************************************
; int2BCD — convert a 16-bit binary value in D to 5 individual BCD digits
; In:  D = unsigned binary value (0-65535)
; Out: TEN_THOUS, THOUSANDS, HUNDREDS, TENS, UNITS filled with digit values 0-9
;***************************************************************************************************
int2BCD           XGDX                    ; Move input value to X for safekeeping
                  LDAA #0
                  STAA TEN_THOUS          ; Zero all output digit registers
                  STAA THOUSANDS
                  STAA HUNDREDS
                  STAA TENS
                  STAA UNITS
                  STAA BCD_SPARE
                  STAA BCD_SPARE+1
                  CPX #0
                  BEQ CON_EXIT            ; Input is zero — nothing to convert
                  XGDX                    ; Restore input to D
                  LDX #10
                  IDIV                    ; D / 10: quotient in X, remainder in D
                  STAB UNITS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB TENS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB HUNDREDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB THOUSANDS
                  CPX #0
                  BEQ CON_EXIT
                  XGDX
                  LDX #10
                  IDIV
                  STAB TEN_THOUS

CON_EXIT          RTS

; LCD_POS_CRSR — move the LCD cursor to DDRAM address in A
LCD_POS_CRSR      ORAA #%10000000         ; Set the "set DDRAM address" command bit
                  JSR cmd2LCD
                  RTS

;***************************************************************************************************
; BIN2ASC — convert an 8-bit value in A to two ASCII hex characters returned in D
; In:  A = byte to convert
; Out: A = ASCII character for upper nibble, B = ASCII character for lower nibble
;***************************************************************************************************
BIN2ASC               PSHA
                      TAB
                      ANDB #%00001111     ; Isolate lower nibble
                      CLRA
                      ADDD #HEX_TABLE     ; Index into hex lookup table
                      XGDX
                      LDAA 0,X            ; Fetch lower nibble ASCII character
                      PULB                ; Restore original byte into B
                      PSHA                ; Save lower nibble character
                      RORB                ; Rotate upper nibble down to bits 3:0
                      RORB
                      RORB
                      RORB
                      ANDB #%00001111     ; Isolate upper nibble
                      CLRA
                      ADDD #HEX_TABLE
                      XGDX
                      LDAA 0,X            ; Fetch upper nibble ASCII character
                      PULB                ; Retrieve lower nibble character into B
                      RTS

;***************************************************************************************************
; BCD2ASC — convert BCD digit registers to ASCII with leading-zero suppression
; Reads TEN_THOUS through UNITS, replaces leading zeros with spaces,
; then ORs remaining digits with $30 to produce ASCII '0'-'9'.
; The UNITS digit is never suppressed.
;***************************************************************************************************
BCD2ASC           LDAA    #0
                  STAA    NO_BLANK        ; Clear the "non-zero seen" flag

C_TTHOU           LDAA    TEN_THOUS
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK1
ISBLANK1          LDAA    #' '
                  STAA    TEN_THOUS
                  BRA     C_THOU
NOT_BLANK1        LDAA    TEN_THOUS
                  ORAA    #$30
                  STAA    TEN_THOUS
                  LDAA    #$1
                  STAA    NO_BLANK

C_THOU            LDAA    THOUSANDS
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK2
ISBLANK2          LDAA    #' '
                  STAA    THOUSANDS
                  BRA     C_HUNS
NOT_BLANK2        LDAA    THOUSANDS
                  ORAA    #$30
                  STAA    THOUSANDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_HUNS            LDAA    HUNDREDS
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK3
ISBLANK3          LDAA    #' '
                  STAA    HUNDREDS
                  BRA     C_TENS
NOT_BLANK3        LDAA    HUNDREDS
                  ORAA    #$30
                  STAA    HUNDREDS
                  LDAA    #$1
                  STAA    NO_BLANK

C_TENS            LDAA    TENS
                  ORAA    NO_BLANK
                  BNE     NOT_BLANK4
ISBLANK4          LDAA    #' '
                  STAA    TENS
                  BRA     C_UNITS
NOT_BLANK4        LDAA    TENS
                  ORAA    #$30
                  STAA    TENS

C_UNITS           LDAA    UNITS           ; Units digit is always converted (never blanked)
                  ORAA    #$30
                  STAA    UNITS
                  RTS

;***************************************************************************************************
; INTERRUPT VECTORS
;***************************************************************************************************
                  ORG     $FFFE
                  DC.W    Entry           ; Reset vector — jump to program entry on power-up/reset

                  ORG     $FFDE
                  DC.W    TOF_ISR         ; Timer Overflow vector — fires at ~23 Hz