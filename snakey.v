module snakey(
    CLOCK_50,
    KEY,
    SW,
   	PS2_CLK,
   	PS2_DAT,
		HEX0,
    // The ports below are for the VGA output.
    VGA_CLK,       // VGA Clock
    VGA_HS,        // VGA H_SYNC
    VGA_VS,        // VGA V_SYNC
    VGA_BLANK_N,   // VGA BLANK
    VGA_SYNC_N,    // VGA SYNC
    VGA_R,         // VGA Red   [9:0]
    VGA_G,         // VGA Green [9:0]
    VGA_B          // VGA Blue  [9:0]
   );
  input CLOCK_50;
  input [9:0] SW;
  input [3:0] KEY;
  output [6:0] HEX0;
  inout PS2_CLK;
  inout PS2_DAT;
  // Outputs for VGA
  output VGA_CLK;      // VGA Clock
  output VGA_HS;       // VGA H_SYNC
  output VGA_VS;       // VGA V_SYNC
  output VGA_BLANK_N;  // VGA BLANK
  output VGA_SYNC_N;   // VGA SYNC
  output [9:0] VGA_R;  // VGA Red[9:0]
  output [9:0] VGA_G;  // VGA Green[9:0]
  output [9:0] VGA_B;  // VGA Blue[9:0]


  wire [2:0] colour;
  wire [7:0] x;
  wire [6:0] y;
  wire writeEn;
  
  vga_adapter VGA(
    .resetn(resetn),
    .clock(CLOCK_50),
    .colour(colour),
    .x(x),
    .y(y),
    .plot(writeEn),
    /* Signals for the DAC to drive the monitor. */
    .VGA_R(VGA_R),
    .VGA_G(VGA_G),
    .VGA_B(VGA_B),
    .VGA_HS(VGA_HS),
    .VGA_VS(VGA_VS),
    .VGA_BLANK(VGA_BLANK_N),
    .VGA_SYNC(VGA_SYNC_N),
    .VGA_CLK(VGA_CLK)
  );
  defparam VGA.RESOLUTION = "160x120";
  defparam VGA.MONOCHROME = "FALSE";
  defparam VGA.BITS_PER_COLOUR_CHANNEL = 1;
  defparam VGA.BACKGROUND_IMAGE = "black.mif";

  wire resetn;
  assign resetn = KEY[0];
  wire up, down, left, right;
  wire [2:0] state;
  wire [1:0] direction;
  wire [7:0] score;
  
  keyboard_tracker #(.PULSE_OR_HOLD(0)) keyboard(.clock(CLOCK_50),
							.reset(resetn),
  							.PS2_CLK(PS2_CLK),
  							.PS2_DAT(PS2_DAT),
  							.left(left),
  							.right(right),
  							.up(up),
  							.down(down));
  							
  wire veryslow, slow, fast, veryfast;
  rate_divider rd1(CLOCK_50, 26'd12999999, 	veryslow); 	// slower
  rate_divider rd2(CLOCK_50, 26'd6499999, 	slow); 		// slow
  rate_divider rd3(CLOCK_50, 26'd4249999, 	fast); 		// fast-ish
  rate_divider rd4(CLOCK_50, 26'd699999,  	veryfast); 	// fast!

  wire speed;
  mux4to1 gameMux(
    .x0(veryslow),
    .x1(slow),
    .x2(fast),
    .x3(veryfast),
    .s0(SW[2]),
    .s1(SW[3]),
    .out(speed)
  );

  controller ctrl(
    .clkin(CLOCK_50),
    .speed(speed),
    .resetn(resetn),
    .left(left),
    .up(up),
    .down(down),
    .right(right),
    .direction(direction),
    .state(state),
    .writeEn(writeEn),
	.gameover(gameover)
  );

  datapath data(
    .clkin(CLOCK_50),
    .resetn(resetn),
    .speed(speed),
    .direction(direction),
    .state(state),
    .walls(SW[8:7]),
    .x(x),
    .y(y),
    .colour(colour),
    .score(score),
	.gameover(gameover)
  );
  hex_decoder score_hex(score[3:0], HEX0);
endmodule

module controller(
  input clkin,
  input resetn,		// active low
  input speed,
  input left, up, down, right,
  input gameover,
  output reg [1:0] direction,
  output [2:0] state,
  output reg writeEn
  );
  
  reg [2:0] current_state, next_state;
  
  localparam	INITIAL_	= 3'b000,
  				UPDATE_HEAD	= 3'b001,
				ERASE_TAIL	= 3'b010,
				MOVE       	= 3'b100,
			  	CHECK_WALLS	= 3'b101,
              	DRAW_APPLE  = 3'b110,
	            END_CYCLE   = 3'b111;
	            
  localparam  LEFT  = 2'b00,
              UP    = 2'b01,
              DOWN  = 2'b10,
              RIGHT = 2'b11;
              
  initial begin
    direction <= RIGHT;
    current_state <= INITIAL_;
    writeEn <= 1'b0;
  end
  always @(posedge clkin)
  begin: state_table
    case (current_state)
	  INITIAL_:			next_state <= speed ? UPDATE_HEAD : INITIAL_;
      UPDATE_HEAD:  	next_state <= ERASE_TAIL;
      ERASE_TAIL:       next_state <= MOVE;
      MOVE:        		next_state <= CHECK_WALLS;
      CHECK_WALLS:		next_state <= DRAW_APPLE;
      DRAW_APPLE:      	next_state <= END_CYCLE;
      END_CYCLE:        next_state <= speed ? INITIAL_: END_CYCLE;
      default:          next_state <= END_CYCLE;
    endcase
  end
  
  always @(posedge clkin) begin
    if (~resetn) begin                       // if resetn is low,
      direction <= RIGHT;                    // set direction to right
    end else begin
      if (left && direction != RIGHT)  // if player presses left,
        direction <= LEFT;                   // set direction to left
      if (up && direction != DOWN)     // if player presses up,
        direction <= UP;                     // set direction to up
      if (down && direction != UP)     // if player presses down,
        direction <= DOWN;                   // set direction to down
      if (right && direction != LEFT)  // if player presses right,
        direction <= RIGHT;                  // set direction to right
    end
  end
  // set the current state
  always @(posedge clkin) begin
    if (~resetn || gameover) begin
      current_state <= INITIAL_;
      writeEn <= 1'b0;
    end else begin
      current_state <= next_state;
      writeEn <= 1'b1;
    end
  end
  assign state = current_state;

endmodule

//////////////////////////////////////////////////////////////////////////////

module datapath(
  input clkin,
  input resetn,
  input speed,
  input [1:0] direction,
  input [2:0] state,
  input [1:0] walls,
  output reg [7:0] score,
  output reg [7:0] x,
  output reg [6:0] y,
  output reg [2:0] colour,
  output reg gameover
);
  reg [7:0] headX, appleX;
  reg [6:0] headY, appleY;
  reg [7:0] bodyX[0:127];
  reg [6:0] bodyY[0:127];
  reg if_init;
  integer i;

  initial begin
    colour <= BLACK;
    headX <= 80; bodyX[0] <= 80; x <= 80;
    headY <= 60; bodyY[0] <= 60; y <= 60;
    appleX <= 50; appleY <= 50;
    score <= 0; gameover <= 0; if_init <= 1;
  end

  // constants for FSM stategameclks
  localparam	INITIAL_	= 3'b000,
  				UPDATE_HEAD = 3'b001,
				ERASE_TAIL	= 3'b010,
				MOVE       	= 3'b100,
			  	CHECK_WALLS	= 3'b101,
              	DRAW_APPLE  = 3'b110,
	            END_CYCLE   = 3'b111;
  localparam  UP  	= 2'b00,
              DOWN  = 2'b01,
              LEFT  = 2'b10,
              RIGHT = 2'b11;
  // constants for colours
  localparam  BLACK  = 3'b000,
              RED    = 3'b100,
              GREEN  = 3'b010,
              PURPLE = 3'b101;
	reg enable1, enable2;
	wire [8:0] q1, q2;
	counter cntr1(clkin, resetn, enable1, q1, 9'b001111111); //[6:3] [2:0]
	counter cntr2(clkin, resetn, enable2, q2, 9'b111111111); //[8:6] [5:0]
	wire [7:0] randX; wire [6:0] randY;
  	random_coordinates coordinates(
    	.clkin(clkin),
    	.randX(randX),
    	.randY(randY)
  	);
	
  always @(posedge clkin) begin
    if (~resetn || gameover) begin
      colour <= BLACK;
      headX <= 80; bodyX[0] <= 80; 
      headY <= 60; bodyY[0] <= 60;
      appleX <= 50; appleY <= 50;
      x <= appleX; y <= appleY;
      score <= 0; gameover <= 0; if_init <= 1;
	  enable1 <= 0; enable2 <= 0;
    end

	if (state == INITIAL_) begin
		colour <= PURPLE;
		case(walls)
			2'b01: begin
				enable1 <= 1;
				x <= 20 + q1[6:3]; // 20 ~ 35
				y <= 20 + q1[2:0]; // 20 ~ 27
			end
			2'b10: begin
				enable2 <= 1;
				x <= 120 + q2[8:6]; // 120 ~ 127
				y <= 28 + q2[5:0];  // 28 ~ 91
			end
			default: begin
				x <= 161; y <= 121;
			end
		endcase
	
	end else if (state == UPDATE_HEAD) begin
		enable1 <= 0; enable2 <= 0;
      case (direction)
        LEFT: begin
			if (headX == 250) begin gameover <= 1; end
        	else begin headX <= headX - 1'b1; end
        	end
        UP: begin
			if (headY == 124) begin gameover <= 1; end
        	else begin headY <= headY - 1'b1; end
        end
        DOWN: begin
			if (headY == 126) begin gameover <= 1; end
			else begin headY <= headY + 1'b1; end
        end
        RIGHT: begin
			if (headX == 164) begin gameover <= 1; end
        	else begin headX <= headX + 1'b1; end
        end
      endcase

	end else if (state == ERASE_TAIL) begin
      x <= bodyX[score]; y <= bodyY[score];
      if (appleX == headX && appleY == headY) begin
        colour <= GREEN;
        score <= score + 1;
      end else begin
		colour <= BLACK;
      	end

	end else if (state == MOVE) begin
        colour <= GREEN;
      	x <= headX; y <= headY;
      	bodyX[0] <= headX; bodyY[0] <= headY;
      	for (i = 0; i < 127; i = i + 1) begin
				bodyX[i + 1'b1] <= bodyX[i];
				bodyY[i + 1'b1] <= bodyY[i];
      	end
      
    end else if (state == CHECK_WALLS) begin
    	case (walls)
			2'b01 : begin
				if ((headX > 19) && (headX < 36) && (headY > 19) && (headY < 28)) begin
					gameover <= 1;
				end
				end
			2'b10 : begin
				if ((headX > 119) && (headX < 128) && (headY > 27) && (headY < 92)) begin
					gameover <= 1;
				end
				end
			default : gameover <= 0;
		endcase
				
	end else if (state == DRAW_APPLE) begin
      if (appleX == headX && appleY == headY || if_init) begin
        if_init <= 0;
        if (randX == 0 || randX == 159 || randY <= 0 || randY == 119) begin
          if_init <= 1;
          appleX <= 255;
          appleY <= 127;
          x <= 255;
          y <= 127;
        end else begin
          appleX <= randX;
          appleY <= randY;
          x <= randX;
          y <= randY;
          colour <= RED;
        end
      end
    end
  end	
endmodule


module counter(clkin, reset_n, enable, q, d);
	input 		clkin, reset_n, enable;
	input [8:0]	d;
	output reg 	[8:0] 	q;
	
	always @(posedge clkin) begin
		if (reset_n == 1'b0) begin
			q <= 9'd0;
			end
		else if (enable == 1'b1) begin
		  if (q == d) begin
			  q <= 9'd0;
			  end
		  else begin
			  q <= q + 1'b1;
			  end
		end
   end
endmodule

module random_coordinates(
   input clkin,
   output reg [7:0] randX,
   output reg [6:0] randY
   );
   initial begin
      randX <= 8'b10101010;
      randY <= 7'b0011001;
   end
   always @(posedge clkin) begin
      randX <= ((randX + 4) % 154) + 1;
      randY <= ((randY + 30) % 116) + 1;
   end
endmodule // random_coordinates


//module random_coordinates(
//   input clkin,
//   input walls,
//   output reg [7:0] randX,
//   output reg [6:0] randY
//   );
//   reg [7:0] tempX;
//   reg [6:0] tempY;
//   reg change;
//   initial begin
//      randX <= 8'b10101010;
//      randY <= 7'b0011001; change <= 1'b0;
//   end
//   always @(posedge clkin) begin
//      tempX <= ((tempX + 4) % 154) + 1;
//      tempY <= ((tempY + 30) % 116) + 1;   
//      case (walls)
//      	2'b01: begin
//      			if ((tempX > 19) && (tempX < 36) && (tempY > 19) && (tempY < 28))
//					begin change <= 1'b1; end
//      		   end
//      	2'b10: begin
//      			if ((tempX > 119) && (tempX < 128) && (tempY > 27) && (tempY < 92))
//    	  			begin change <= 1'b1; end
//      		   end
//      endcase
//      if 		(tempX < 2) 	begin 	tempX <= tempX + 27; 	end
//      else if 	(tempX > 158) 	begin 	tempX <= tempX - 27;	end
//      if 		(tempY < 2) 	begin 	tempX <= tempX + 17; 	end
//      else if 	(tempY > 118) 	begin 	tempX <= tempX - 37; 	end
//      randX <= tempX;
//      randY <= tempY;
//   end
//endmodule

module rate_divider(clkin, d, out);
	input clkin, reset, parload_, enable;
	input [25:0] d;
	reg [25:0] q;
	output reg out;

	initial begin
    	clkout <= 1'b0;
    	q <= 26'b0;
  	end

	always @(posedge clkin)
	begin
		if (q == {28{1'b0}}) begin
			q <= d;
		end else begin
			q <= q - 1'b1;
		end
	end
	assign out = (q == {28{1'b0}}) ? 1'b1 : 1'b0;
endmodule

module mux2to1(
  input x, y, s,
  output m
  );
	assign m = x & ~s | y & s;
endmodule 

module mux4to1(
	input x0, x1, x2, x3, s0, s1,
	output out
	);
	wire out01, out23;
	mux2to1 m0( .x(x0), .y(x1), .s(s0), .m(out01) );
	mux2to1 m1( .x(x2), .y(x3), .s(s0), .m(out23) );
	mux2to1 m2( .x(out01), .y(out23), .s(s1), .m(out) );
endmodule

module hex_decoder(hex_digit, segments);
    input [3:0] hex_digit;
    output reg [6:0] segments;
   
    always @(*)
        case (hex_digit)
            4'h0: segments = 7'b100_0000;
            4'h1: segments = 7'b111_1001;
            4'h2: segments = 7'b010_0100;
            4'h3: segments = 7'b011_0000;
            4'h4: segments = 7'b001_1001;
            4'h5: segments = 7'b001_0010;
            4'h6: segments = 7'b000_0010;
            4'h7: segments = 7'b111_1000;
            4'h8: segments = 7'b000_0000;
            4'h9: segments = 7'b001_1000;
            4'hA: segments = 7'b000_1000;
            4'hB: segments = 7'b000_0011;
            4'hC: segments = 7'b100_0110;
            4'hD: segments = 7'b010_0001;
            4'hE: segments = 7'b000_0110;
            4'hF: segments = 7'b000_1110;   
            default: segments = 7'h7f;
        endcase
endmodule
