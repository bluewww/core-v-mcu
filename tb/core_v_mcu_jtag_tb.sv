// Copyright 2022 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

// tb_pulp.sv
// Francesco Conti <fconti@iis.ee.ethz.ch>
// Antonio Pullini <pullinia@iis.ee.ethz.ch>
// Igor Loi <igor.loi@unibo.it>
// Robert Balas <balasr@iis.ee.ethz.ch>


module core_v_mcu_jtag_tb;
  import srec_pkg::*;

  parameter CONFIG_FILE = "NONE";

  // simulation platform parameters

 // if we are using a simulated stdout
  parameter SIM_STDOUT = 1;

  // period of the external reference clock (32.769kHz)
  parameter REF_CLK_PERIOD = 30517ns;

  // UART baud rate in bps
  parameter BAUDRATE = 115200;

  // exit
  localparam int EXIT_SUCCESS = 0;
  localparam int EXIT_FAIL = 1;
  localparam int EXIT_ERROR = -1;

  // simulation variables & flags
  string bootmode;
  logic uart_tb_rx_en = 1'b0;

  // contains the program code
  string stimuli_path, srec_path;
  int num_stim;
  logic [95:0] stimuli[$];  // array for the stimulus vectors

  logic dev_dpi_en = 0;

  logic [255:0][31:0] jtag_data;

  int exit_status = EXIT_ERROR;  // modelsim exit code, will be overwritten when successfull

  jtag_pkg::test_mode_if_t test_mode_if = new;
  jtag_pkg::debug_mode_if_t debug_mode_if = new;
  pulp_tap_pkg::pulp_tap_if_soc_t pulp_tap = new;

  // system wires
  // the w_/s_ prefixes are used to mean wire/tri-type and logic-type (respectively)

  logic s_rst_n = 1'b0;
  logic s_rst_dpi_n;
  wire w_rst_n;

  logic s_clk_ref;
  wire w_clk_ref;

  tri w_spi_master_sdio0;
  tri w_spi_master_sdio1;
  tri w_spi_master_sdio2;
  tri w_spi_master_sdio3;
  tri w_spi_master_csn0;
  tri w_spi_master_csn1;
  tri w_spi_master_sck;

  tri w_sdio_data0;

  wire w_i2c0_scl;
  wire w_i2c0_sda;

  tri w_i2c1_scl;
  tri w_i2c1_sda;

  tri w_uart_rx;
  tri w_uart_tx;

  wire w_cam_pclk;
  wire [7:0] w_cam_data;
  wire w_cam_hsync;
  wire w_cam_vsync;

  // I2S 0
  wire w_i2s0_sck;
  wire w_i2s0_ws;
  wire w_i2s0_sdi;
  // I2S 1
  wire w_i2s1_sdi;

  wire w_i2s_sck;
  wire w_i2s_ws;
  wire [7:0] w_i2s_data;

  wire w_trstn;
  wire w_tck;
  wire w_tdi;
  wire w_tms;
  wire w_tdo;

  logic s_vpi_trstn;
  logic s_vpi_tck;
  logic s_vpi_tdi;
  logic s_vpi_tms;

  wire w_bridge_trstn;
  wire w_bridge_tdo;
  wire w_bridge_tck;
  wire w_bridge_tdi;
  wire w_bridge_tms;

  logic s_trstn = 1'b0;
  logic s_tck = 1'b0;
  logic s_tdi = 1'b0;
  logic s_tms = 1'b0;
  logic s_tdo;

  // jtag openocd bridge signals
  logic sim_jtag_tck;
  logic sim_jtag_tms;
  logic sim_jtag_tdi;
  logic sim_jtag_trstn;
  logic sim_jtag_tdo;
  logic [31:0] sim_jtag_exit;
  logic sim_jtag_enable;

  // tmp signals for assignment to wires
  logic tmp_rst_n;
  logic tmp_clk_ref;
  logic tmp_trstn;
  logic tmp_tck;
  logic tmp_tdi;
  logic tmp_tms;
  logic tmp_tdo;
  logic tmp_bridge_tdo;

  wire w_master_i2s_sck;
  wire w_master_i2s_ws;

  wire w_bootsel;
  logic s_bootsel;

  logic [8:0] jtag_conf_reg, jtag_conf_rego;  //22bits but actually only the last 9bits are used

  always_comb begin
    sim_jtag_enable = 1'b0;

    if ($test$plusargs("jtag_openocd")) begin
      tmp_rst_n       = s_rst_n;
      tmp_clk_ref     = s_clk_ref;
      tmp_trstn       = sim_jtag_trstn;
      tmp_tck         = sim_jtag_tck;
      tmp_tdi         = sim_jtag_tdi;
      tmp_tms         = sim_jtag_tms;
      tmp_tdo         = w_tdo;
      tmp_bridge_tdo  = w_tdo;
      sim_jtag_enable = 1'b1;
    end else begin
      tmp_rst_n      = s_rst_n;
      tmp_clk_ref    = s_clk_ref;

      tmp_trstn      = s_trstn;
      tmp_tck        = s_tck;
      tmp_tdi        = s_tdi;
      tmp_tms        = s_tms;
      tmp_tdo        = w_tdo;
      tmp_bridge_tdo = w_tdo;
    end
  end

  assign w_rst_n      = tmp_rst_n;
  assign w_clk_ref    = tmp_clk_ref;
  assign s_cam_valid  = 1'b0;
  assign w_trstn      = tmp_trstn;
  assign w_tck        = tmp_tck;
  assign w_tdi        = tmp_tdi;
  assign w_tms        = tmp_tms;
  assign s_tdo        = tmp_tdo;
  assign w_bridge_tdo = tmp_bridge_tdo;
  assign sim_jtag_tdo = tmp_tdo;


  if (CONFIG_FILE == "NONE") begin
    assign w_uart_tx = w_uart_rx;
  end


  assign w_bootsel = s_bootsel;

  // UART receiver
  uart_sim #(
    .BAUD_RATE(BAUDRATE),
    .PARITY_EN(0)
  ) i_uart_sim (
    .rx       (w_uart_rx),
    .rx_en    (uart_tb_rx_en),
    .tx()
  );

  // jtag calls from dpi
  SimJTAG #(
    .TICK_DELAY(1),
    .PORT      (4567)
  ) i_sim_jtag (
    .clock          (w_clk_ref),
    .reset          (~s_rst_n),
    .enable         (sim_jtag_enable),
    .init_done      (s_rst_n),
    .jtag_TCK       (sim_jtag_tck),
    .jtag_TMS       (sim_jtag_tms),
    .jtag_TDI       (sim_jtag_tdi),
    .jtag_TRSTn     (sim_jtag_trstn),
    .jtag_TDO_data  (sim_jtag_tdo),
    .jtag_TDO_driven(1'b1),
    .exit           (sim_jtag_exit)
  );

  core_v_mcu i_dut (
    .jtag_tck_i(w_tck),
    .jtag_tdi_i(w_tdi),
    .jtag_tdo_o(w_tdo),
    .jtag_tms_i(w_tms),
    .jtag_trst_i(w_trstn),
    .ref_clk_i(w_clk_ref),
    .rstn_i(w_rst_n),
    .bootsel_i(w_bootsel),
    .stm_i(1'b0),
    .io_in_i('0),
    .io_out_o(),
    .pad_cfg_o(),
    .io_oe_o()
  );

  initial forever #(REF_CLK_PERIOD/2) s_clk_ref=~s_clk_ref;

  initial begin : timing_format
    $timeformat(-9, 0, "ns", 9);
  end

  // testbench driver process
  initial begin

    logic [1:0]  dm_op;
    logic [6:0]  dm_addr;
    logic [31:0] dm_data;
    logic error;
    int   num_err;
    int   rd_cnt;

    int          entry_point;
    logic [31:0] begin_l2_instr;
    automatic logic [9:0] FC_CORE_ID;
    automatic srec_record_t records[$];
    automatic string jtag_tap_type;

    FC_CORE_ID    = {5'd31, 5'd0};
    uart_tb_rx_en = 1'b1;  // enable uart rx in testbench
    error         = 1'b0;
    num_err       = 0;
    rd_cnt        = 0;

    // read boot mode from commandline
    // read entry point from commandline
    if (!$value$plusargs("bootmode=%s", bootmode))
      bootmode = "jtag";

    if ($value$plusargs("ENTRY_POINT=%h", entry_point))
      begin_l2_instr = entry_point;
    else
      begin_l2_instr = 32'h1C008080;

    $display("[TB  ] %t - Asserting hard reset", $realtime);
    s_rst_n = 1'b0;

    #1ns;


    if ($test$plusargs("jtag_openocd")) begin
      // Use openocd to interact with the simulation
      s_bootsel = 1'b0;
      $display("[TB  ] %t - Releasing hard reset", $realtime);
      s_rst_n = 1'b1;

    end else begin
      // Use only the testbench to do the loading and running

      // determine if we want to load the binary with jtag or from flash
      if (bootmode == "spi_flash") begin
        // jtag reset needed anyway
        jtag_pkg::jtag_reset(s_tck, s_tms, s_trstn, s_tdi);
        jtag_pkg::jtag_softreset(s_tck, s_tms, s_trstn, s_tdi);
        #5us;

        s_bootsel = 1'b1;

        $display("[TB  ] %t - Releasing hard reset", $realtime);
        s_rst_n = 1'b1;
        debug_mode_if.init_dmi_access(s_tck, s_tms, s_trstn, s_tdi);
        debug_mode_if.set_dmactive(1'b1, s_tck, s_tms, s_trstn, s_tdi, s_tdo);
        #10us;

      end else if (bootmode == "jtag") begin
        s_bootsel = 1'b0;
      end else begin
        $error("Unknown bootmode: %s", bootmode);
      end

      if (bootmode == "jtag") begin
        // read in the stimuli vectors  == address_value
        // we support two formats:
        //
        // 1. stim.txt where each text line is 96 bits encoded in ascii. The
        // first 32 bits are the address the remaining 64 bits the data payload
        // 2. *.srec. Srecords is a standardized format to represent binary data
        // in ascii text format. Notably, it also encodes also the entry point
        // so we don't have to supply it manully with +ENTRY_POINT. GNU objcopy
        // (part of binutils) can easily convert and elf file to this format.
        if ($value$plusargs("stimuli=%s", stimuli_path)) begin
          $display("[TB  ] %t - Loading custom stimuli from %s", $realtime, stimuli_path);
          load_stim(stimuli_path, stimuli);
        end else if ($value$plusargs("srec=%s", srec_path)) begin
          $display("[TB  ] %t - Loading srec from %s", $realtime, srec_path);
          srec_read(srec_path, records);
          srec_records_to_stimuli(records, stimuli, entry_point);
          if (!$test$plusargs("srec_ignore_entry"))
            begin_l2_instr = entry_point;
        end else begin
          $display("[TB  ] %t - Loading default stimuli", $realtime);
          load_stim("./vectors/stim.txt", stimuli);
        end

        $display("[TB  ] %t - Entry point is set to 0x%h", $realtime, begin_l2_instr);

        // before starting the actual boot procedure we do some light
        // testing on the jtag link
        jtag_pkg::jtag_reset(s_tck, s_tms, s_trstn, s_tdi);
        jtag_pkg::jtag_softreset(s_tck, s_tms, s_trstn, s_tdi);
        #5us;

        jtag_pkg::jtag_bypass_test(s_tck, s_tms, s_trstn, s_tdi, s_tdo);
        #5us;

        jtag_pkg::jtag_get_idcode(s_tck, s_tms, s_trstn, s_tdi, s_tdo);
        #5us;

        test_mode_if.init(s_tck, s_tms, s_trstn, s_tdi);

        $display("[TB  ] %t - Releasing hard reset", $realtime);
        s_rst_n = 1'b1;

        // //test if the PULP tap che write to the L2
        // pulp_tap.init(s_tck, s_tms, s_trstn, s_tdi);

        // $display("[TB  ] %t - Init PULP TAP", $realtime);

        // pulp_tap.write32(begin_l2_instr, 1, 32'hABBAABBA, s_tck, s_tms, s_trstn, s_tdi, s_tdo);

        // $display("[TB  ] %t - Write32 PULP TAP", $realtime);

        // #50us;
        // pulp_tap.read32(begin_l2_instr, 1, jtag_data, s_tck, s_tms, s_trstn, s_tdi, s_tdo);

        // if (jtag_data[0] != 32'hABBAABBA)
        //   $display("[JTAG] %t - R/W test of L2 failed: %h != %h", $realtime, jtag_data[0], 32'hABBAABBA);
        // else $display("[JTAG] %t - R/W test of L2 succeeded", $realtime);

        // From here on starts the actual jtag booting

        // Setup debug module and hart, halt hart and set dpc (return point
        // for boot).
        // Halting the fc hart transfers control of the program execution to
        // the debug module. This might take a bit until the debug request
        // signal is propagated so meanwhile the core is executing stuff
        // from the bootrom. For jtag booting (what we are doing right now),
        // bootsel is low so the code that is being executed in said bootrom
        // is only a busy wait or wfi until the debug unit grabs control.
        debug_mode_if.init_dmi_access(s_tck, s_tms, s_trstn, s_tdi);

        debug_mode_if.set_dmactive(1'b1, s_tck, s_tms, s_trstn, s_tdi, s_tdo);

        debug_mode_if.set_hartsel(FC_CORE_ID, s_tck, s_tms, s_trstn, s_tdi, s_tdo);

        $display("[TB  ] %t - Halting the Core", $realtime);
        debug_mode_if.halt_harts(s_tck, s_tms, s_trstn, s_tdi, s_tdo);

        $display("[TB  ] %t - Writing the boot address into dpc", $realtime);
        debug_mode_if.write_reg_abstract_cmd(riscv::CSR_DPC, begin_l2_instr, s_tck, s_tms, s_trstn,
                                             s_tdi, s_tdo);

        // long debug module + jtag tests
        if ($test$plusargs("jtag_dm_tests")) begin
          debug_mode_if.run_dm_tests(FC_CORE_ID, begin_l2_instr, error, num_err, s_tck, s_tms,
                                     s_trstn, s_tdi, s_tdo);
          // we don't have any program to load so we finish the testing
          if (num_err == 0) begin
            exit_status = EXIT_SUCCESS;
          end else begin
            exit_status = EXIT_FAIL;
            $error("Debug Module: %d tests failed", num_err);
          end
          $stop;
        end

        if (bootmode == "jtag") begin
          $display("[TB  ] %t - Loading L2 via JTAG", $realtime);
          if (!$value$plusargs("jtag_load_tap=%s", jtag_tap_type))
            jtag_tap_type = "riscv"; // default
          if (jtag_tap_type == "riscv") begin
            // use debug module to load binary
            debug_mode_if.load_L2(num_stim, stimuli, s_tck, s_tms, s_trstn, s_tdi, s_tdo);
          end else if (jtag_tap_type == "pulp") begin
            $fatal(1, "pulp tap doesn't exist in core-v-mcu");
          end else begin
            $fatal(1, "Unknown tap type +jtag_load_tap=%s", jtag_tap_type);
          end
        end else begin
          $error("Unknown L2 loading mechnism chosen (bootmode == %s)", bootmode);
        end

        // configure for debug module dmi access again
        debug_mode_if.init_dmi_access(s_tck, s_tms, s_trstn, s_tdi);

        // we have set dpc and loaded the binary, we can go now
        $display("[TB  ] %t - Resuming the CORE", $realtime);
        debug_mode_if.resume_harts(s_tck, s_tms, s_trstn, s_tdi, s_tdo);
      end

      #500us;

      // enable sb access for subsequent readMem calls
      debug_mode_if.set_sbreadonaddr(1'b1, s_tck, s_tms, s_trstn, s_tdi, s_tdo);

      // wait for end of computation signal
      $display("[TB  ] %t - Waiting for end of computation", $realtime);

      jtag_data[0] = 0;
      while (jtag_data[0][31] == 0) begin
        // every 10th loop iteration, clear the debug module's SBA unit CSR to make
        // sure there's no error blocking our reads. Sometimes a TCDM read
        // request issued by the debug module takes longer than it takes
        // for the read request to the debug module to arrive and it
        // stores an error in the SBCS register. By clearing it
        // periodically we make sure the test can terminate.
        if (rd_cnt % 10 == 0) begin
          debug_mode_if.clear_sbcserrors(s_tck, s_tms, s_trstn, s_tdi, s_tdo);
        end
        debug_mode_if.readMem(32'h1A1040A0, jtag_data[0], s_tck, s_tms, s_trstn, s_tdi, s_tdo);
        rd_cnt++;
        #50us;
      end

      if (jtag_data[0][30:0] == 0) exit_status = EXIT_SUCCESS;
      else exit_status = EXIT_FAIL;
      $display("[TB  ] %t - Received status core: 0x%h", $realtime, jtag_data[0][30:0]);

      $stop;

    end
  end

  task load_stim(input string stim, output logic [95:0] stimuli[$]);
    int stim_fd, ret;
    logic [95:0] rdata;
    stim_fd = $fopen(stim, "r");

    if (stim_fd == 0)
      $fatal(1, "Could not open stimuli file!");

    while (!$feof(stim_fd)) begin
      ret= $fscanf(stim_fd, "%h\n", rdata);
      stimuli.push_back(rdata);
    end

    $fclose(stim_fd);
  endtask  // load_stim

endmodule  // core_v_mcu_jtag_tb
