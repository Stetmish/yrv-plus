// ================================================================
//  top.sv for T-Core Board (Terasic MAX 10 10M50DAF484C7G)
//  UART via aux_uart_rx on TMD_D[6], GND on TMD_D[7]
// ================================================================

`define INTEL_VERSION
`define CLK_FREQUENCY (50 * 1000 * 1000)

`include "yrv_mcu.v"

module top
(
  // -------- Clock Inputs (Table 2-3, page 16) --------
  input         adc_clk_10,       // PIN_N5
  input         max10_clk1_50,    // PIN_P11  (main 50 MHz)
  input         max10_clk2_50,    // PIN_N14

  // -------- Push-Buttons (active low, Table 2-4, page 17) --------
  input  [ 1:0] key,              // KEY[0]: PIN_AB9, KEY[1]: PIN_AA9

  // -------- DIP Switches (Table 2-5, page 18) --------
  input  [ 3:0] sw,               // SW[0]: PIN_AB16, SW[1]: PIN_Y16,
                                   // SW[2]: PIN_V16,  SW[3]: PIN_AB17

  // -------- User LEDs (active high, Table 2-6, page 19) --------
  output [ 3:0] led,              // LED[0]: PIN_AA16, LED[1]: PIN_AB15,
                                   // LED[2]: PIN_AA15, LED[3]: PIN_AA14

  // -------- On-board RGB LED (WS2812B, Table 2-8, page 21) --------
  output        rgb_led_data,     // PIN_D19 (OB_LED_RGB_D)

  // -------- TMD GPIO Header (2x6, Table 2-9, page 23) --------
  inout  [ 7:0] tmd_gpio          // TMD_D[0]: PIN_AB8, ... TMD_D[7]: PIN_AA6
);

  // ================================================================
  // 1.  Clock and Reset
  // ================================================================

  wire clk   = max10_clk1_50;
  wire reset = sw[3];
  wire resetb = ~reset;

  // ================================================================
  // 2.  Slow Clock Mode (optional)
  // ================================================================

  wire slow_clk_mode = sw[0];
  logic [22:0] clk_cnt;

  always_ff @(posedge clk or posedge reset)
    if (reset) clk_cnt <= '0;
    else       clk_cnt <= clk_cnt + 1'd1;

  wire muxed_clk_raw = slow_clk_mode ? clk_cnt[22] : clk;
  wire muxed_clk;

  `ifdef SIMULATION
    assign muxed_clk = muxed_clk_raw;
  `else
    global i_global (.in(muxed_clk_raw), .out(muxed_clk));
  `endif

  // ================================================================
  // 3.  Instantiate yrv_mcu Core (unchanged)
  // ================================================================

  wire         ei_req;
  wire         nmi_req   = 1'b0;
  wire         ser_rxd   = 1'b0;   // main UART RX grounded (as in original)
  wire  [15:0] port4_in  = {8'b0, tmd_gpio}; // read all TMD pins as inputs
  wire  [15:0] port5_in  = '0;

  wire         debug_mode;
  wire         ser_clk;
  wire         ser_txd;            // not connected
  wire         wfi_state;
  wire  [15:0] port0_reg;
  wire  [15:0] port1_reg;
  wire  [15:0] port2_reg;
  wire  [15:0] port3_reg;

  // Memory debug bus (retained for future use)
  wire         mem_ready;
  wire  [31:0] mem_rdata;
  wire         mem_lock;
  wire         mem_write;
  wire   [1:0] mem_trans;
  wire   [3:0] mem_ble;
  wire  [31:0] mem_addr;
  wire  [31:0] mem_wdata;
  wire  [31:0] extra_debug_data;

  // -------- Auxiliary UART (BOOT_FROM_AUX_UART) --------
  `ifdef BOOT_FROM_AUX_UART
    wire aux_uart_rx = tmd_gpio[6];   // TMD_D[6] as RX input
    assign tmd_gpio[7] = 1'b0;        // TMD_D[7] as GND
    assign tmd_gpio[6] = 1'bz;        // high impedance for input
  `else
    wire aux_uart_rx = 1'b0;
    // If not used, drive these pins as outputs
    assign tmd_gpio[6] = port0_reg[6];
    assign tmd_gpio[7] = port0_reg[7];
  `endif

  yrv_mcu i_yrv_mcu (
    .clk       ( muxed_clk   ),
    .ei_req    ( ei_req      ),
    .nmi_req   ( nmi_req     ),
    .resetb    ( resetb      ),
    .ser_rxd   ( ser_rxd     ),
    .port4_in  ( port4_in    ),
    .port5_in  ( port5_in    ),
    .debug_mode( debug_mode  ),
    .ser_clk   ( ser_clk     ),
    .ser_txd   ( ser_txd     ),
    .wfi_state ( wfi_state   ),
    .port0_reg ( port0_reg   ),
    .port1_reg ( port1_reg   ),
    .port2_reg ( port2_reg   ),
    .port3_reg ( port3_reg   ),
    .*                         // connects mem_* and extra_debug_data
  );

  // ================================================================
  // 4.  Standard LEDs (4 pcs) driven by lower 4 bits of port3_reg
  // ================================================================

  assign led = port3_reg[3:0];

  // ================================================================
  // 5.  TMD GPIO usage:
  //     - tmd_gpio[0:5] are outputs driven by port0_reg[0:5]
  //     - tmd_gpio[6] is reserved as RX for AUX UART (if enabled)
  //     - tmd_gpio[7] is reserved as GND for AUX UART (if enabled)
  // ================================================================

  // Drive TMD pins 0..5 as outputs (always)
  assign tmd_gpio[0] = port0_reg[0];
  assign tmd_gpio[1] = port0_reg[1];
  assign tmd_gpio[2] = port0_reg[2];
  assign tmd_gpio[3] = port0_reg[3];
  assign tmd_gpio[4] = port0_reg[4];
  assign tmd_gpio[5] = port0_reg[5];

  // Pins 6 and 7 are handled by the `ifdef` block above.

  // ================================================================
  // 6.  WS2812B RGB LED Driver
  // ================================================================

  logic [7:0] r_color, g_color, b_color;
  always_comb begin
    case (sw[2:1])
      2'b00: begin r_color = 8'hFF; g_color = 8'h00; b_color = 8'h00; end
      2'b01: begin r_color = 8'h00; g_color = 8'hFF; b_color = 8'h00; end
      2'b10: begin r_color = 8'h00; g_color = 8'h00; b_color = 8'hFF; end
      2'b11: begin r_color = 8'hFF; g_color = 8'hFF; b_color = 8'hFF; end
    endcase
  end

  localparam NUM_LEDS = 4;
  localparam BITS_PER_LED = 24;
  localparam TOTAL_BITS = NUM_LEDS * BITS_PER_LED;

  reg [TOTAL_BITS-1:0] shift_reg;
  reg [7:0] bit_cnt;
  reg [23:0] delay_cnt;
  reg sending;
  reg rgb_dout;
  reg key0_prev;
  wire key0_pressed = ~key[0] & ~key0_prev;

  always_ff @(posedge clk or posedge reset)
    if (reset) begin
      key0_prev <= 1'b1;
      shift_reg <= '0;
      bit_cnt   <= 0;
      delay_cnt <= 0;
      sending   <= 0;
      rgb_dout  <= 0;
    end else begin
      key0_prev <= ~key[0];
      if (key0_pressed || (delay_cnt == 0 && !sending)) begin
        shift_reg <= {
          {8{g_color}}, {8{r_color}}, {8{b_color}},
          {8{g_color}}, {8{r_color}}, {8{b_color}},
          {8{g_color}}, {8{r_color}}, {8{b_color}},
          {8{g_color}}, {8{r_color}}, {8{b_color}}
        };
        bit_cnt   <= TOTAL_BITS;
        sending   <= 1;
        delay_cnt <= 0;
      end
      if (sending) begin
        if (bit_cnt > 0) begin
          rgb_dout <= shift_reg[TOTAL_BITS-1];
          shift_reg <= shift_reg << 1;
          bit_cnt   <= bit_cnt - 1;
        end else begin
          rgb_dout <= 0;
          sending <= 0;
          delay_cnt <= 15000;
        end
      end else begin
        if (delay_cnt > 0) delay_cnt <= delay_cnt - 1;
        rgb_dout <= 0;
      end
    end

  assign rgb_led_data = rgb_dout;

  // ================================================================
  // 7.  8 kHz Timer Interrupt (as in original)
  // ================================================================

  logic [12:0] khz8_reg;
  logic        khz8_lat;
  assign ei_req = khz8_lat;
  wire khz8_lim = khz8_reg == 13'd6249;

  always_ff @(posedge clk or negedge resetb)
    if (~resetb) begin
      khz8_reg <= 13'd0;
      khz8_lat <= 1'b0;
    end else begin
      khz8_reg <= khz8_lim ? 13'd0 : khz8_reg + 1'b1;
      khz8_lat <= ~port3_reg[15] & (khz8_lim | khz8_lat);
    end

endmodule