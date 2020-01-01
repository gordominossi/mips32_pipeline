-- mips.vhd
-- From Section 7.6 of Digital Design & Computer Architecture
-- Updated to VHDL 2008 26 July 2011 David_Harris@hmc.edu

library IEEE;
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;

entity testbench is
end;

architecture test of testbench is
  component top
    port(clk, reset:           in  STD_LOGIC;
         writedata, dataadr:   out STD_LOGIC_VECTOR(31 downto 0);
         memwrite:             out STD_LOGIC);
  end component;
  signal writedata, dataadr:    STD_LOGIC_VECTOR(31 downto 0);
  signal clk, reset,  memwrite: STD_LOGIC;
begin

  -- instantiate device to be tested
  dut: top port map(clk, reset, writedata, dataadr, memwrite);

  -- Generate clock with 10 ns period
  process begin
    clk <= '1';
    wait for 5 ns;
    clk <= '0';
    wait for 5 ns;
  end process;

  -- Generate reset for first two clock cycles
  process begin
    reset <= '1';
    wait for 22 ns;
    reset <= '0';
    wait;
  end process;

  -- check that 7 gets written to address 84 at end of program
  process (clk) begin
    if (clk'event and clk = '0' and memwrite = '1') then
      if (to_integer(dataadr) = 84 and to_integer(writedata) = 7) then
        report "NO ERRORS: Simulation succeeded" severity failure;
      elsif (dataadr /= 80) then
        report "Simulation failed" severity failure;
      end if;
    end if;
  end process;
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;

entity top is -- top-level design for testing
  port(clk, reset:           in     STD_LOGIC;
       writedata, dataadr:   buffer STD_LOGIC_VECTOR(31 downto 0);
       memwrite:             buffer STD_LOGIC);
end;

architecture test of top is
  component mips
    port(clk, reset:        in  STD_LOGIC;
         pc:                out STD_LOGIC_VECTOR(31 downto 0);
         instr:             in  STD_LOGIC_VECTOR(31 downto 0);
         memwrite:          out STD_LOGIC;
         aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
         readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component imem
    port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
         rd: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component dmem
    port(clk, we:  in STD_LOGIC;
         a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
         rd:       out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  signal pc, instr,
         readdata: STD_LOGIC_VECTOR(31 downto 0);
begin
  -- instantiate processor and memories
  mips1: mips port map(clk, reset, pc, instr, memwrite, dataadr,
                       writedata, readdata);
  imem1: imem port map(pc(7 downto 2), instr);
  dmem1: dmem port map(clk, memwrite, dataadr, writedata, readdata);
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use STD.TEXTIO.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity dmem is -- data memory
  port(clk, we:  in STD_LOGIC;
       a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
       rd:       out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of dmem is
begin
  process is
    type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
    variable mem: ramtype;
  begin
    -- read or write memory
    loop
      if clk'event and clk = '1' then
          if (we = '1') then mem(to_integer(a(7 downto 2))) := wd;
          end if;
      end if;
      rd <= mem(to_integer(a(7 downto 2)));
      wait on clk, a;
    end loop;

  end process;
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use STD.TEXTIO.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity imem is -- instruction memory
  port(a:  in  STD_LOGIC_VECTOR(5 downto 0);
       rd: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of imem is
begin
  process is
    file mem_file: TEXT;
    variable L: line;
    variable ch: character;
    variable index, result: integer;
    type ramtype is array (63 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
    variable mem: ramtype;
  begin
    -- initialize memory from file
    for i in 0 to 63 loop -- set all contents low
      mem(i) := (others => '0');
    end loop;
    index := 0;
    FILE_OPEN(mem_file, "memfile.dat", READ_MODE);
    while not endfile(mem_file) loop
      readline(mem_file, L);
      result := 0;
      for i in 1 to 8 loop
        read(L, ch);
        if '0' <= ch and ch <= '9' then
            result := character'pos(ch) - character'pos('0');
        elsif 'a' <= ch and ch <= 'f' then
           result := character'pos(ch) - character'pos('a')+10;
        else report "Format error on line " & integer'image(index)
             severity error;
        end if;
        mem(index)(35-i*4 downto 32-i*4) :=to_std_logic_vector(result,4);
      end loop;
      index := index + 1;
    end loop;

    -- read memory
    loop
      rd <= mem(to_integer(a));
      wait on a;
    end loop;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mips is -- single cycle MIPS processor
  port(clk, reset:        in  STD_LOGIC;
       pc:                out STD_LOGIC_VECTOR(31 downto 0);
       instr:             in  STD_LOGIC_VECTOR(31 downto 0);
       memwrite:          out STD_LOGIC;
       aluout, writedata: out STD_LOGIC_VECTOR(31 downto 0);
       readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
end;

architecture struct of mips is
  component controller
    port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
         zero:               in  STD_LOGIC;
         memtoreg, memwrite: out STD_LOGIC;
         alusrc:             out STD_LOGIC;
         regdst, regwrite:   out STD_LOGIC;
         jump:               out STD_LOGIC;
         Branch, Branch_NE:  out STD_LOGIC;
         alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
  end component;
  component datapath
    port(clk, reset:        in  STD_LOGIC;
      memtoreg:          in  STD_LOGIC;
      alusrc, regdst:    in  STD_LOGIC;
      regwrite, jump:    in  STD_LOGIC;
      Branch, Branch_NE: in  STD_LOGIC;
      MemWrite:          in  STD_LOGIC;
      alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
      zero:              out STD_LOGIC;
      pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
      instr:             in  STD_LOGIC_VECTOR(31 downto 0);
      aluout, writedata: buffer STD_LOGIC_VECTOR(31 downto 0);
      readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
  end component;
  signal memtoreg, alusrc, regdst, regwrite, jump, Branch, Branch_NE: STD_LOGIC;
  signal zero: STD_LOGIC;
  signal alucontrol: STD_LOGIC_VECTOR(2 downto 0);
begin
  cont: controller port map(instr(31 downto 26), instr(5 downto 0),
                            zero, memtoreg, memwrite, alusrc,
                            regdst, regwrite, jump, Branch, Branch_NE, alucontrol);
  dp: datapath port map(clk, reset, memtoreg, alusrc, regdst,
                        regwrite, jump, Branch, Branch_NE, memwrite, alucontrol, zero, pc, instr,
                        aluout, writedata, readdata);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity controller is -- single cycle control decoder
  port(op, funct:          in  STD_LOGIC_VECTOR(5 downto 0);
       zero:               in  STD_LOGIC;
       memtoreg, memwrite: out STD_LOGIC;
       alusrc:             out STD_LOGIC;
       regdst, regwrite:   out STD_LOGIC;
       jump:               out STD_LOGIC;
       Branch, Branch_NE:  out STD_LOGIC;
       alucontrol:         out STD_LOGIC_VECTOR(2 downto 0));
end;


architecture struct of controller is
  component maindec
    port(op:                  in  STD_LOGIC_VECTOR(5 downto 0);
      RegWrite, RegDst:    out STD_LOGIC;
      ALUSrc, Branch:      out STD_LOGIC;
      Branch_NE, MemWrite: out STD_LOGIC;
      MemToReg, Jump:      out STD_LOGIC;
      ALUOp:               out STD_LOGIC_VECTOR(1 downto 0));
  end component;
  component aludec
    port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
         aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
         alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
  end component;
  signal aluop:  STD_LOGIC_VECTOR(1 downto 0);
begin
  md: maindec port map(Op, RegWrite, RegDst, ALUSrc, Branch, Branch_NE,
                       MemWrite, MemToReg, Jump, ALUOp);
  ad: aludec port map(funct, aluop, alucontrol);

end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity maindec is -- main control decoder
  port(op:                  in  STD_LOGIC_VECTOR(5 downto 0);
       RegWrite, RegDst:    out STD_LOGIC;
       ALUSrc, Branch:      out STD_LOGIC;
       Branch_NE, MemWrite: out STD_LOGIC;
       MemToReg, Jump:      out STD_LOGIC;
       ALUOp:               out STD_LOGIC_VECTOR(1 downto 0));
end;

architecture behave of maindec is
  signal controls: STD_LOGIC_VECTOR(9 downto 0);
begin
  process(all) begin
    case op is
      when "000000" => controls <= "1100000011"; -- RTYPE
      when "100011" => controls <= "1010001000"; -- LW
      when "101011" => controls <= "0010010000"; -- SW
      when "000100" => controls <= "0001000001"; -- BEQ
      when "000101" => controls <= "0000100001"; -- BNE
      when "001000" => controls <= "1010000000"; -- ADDI
      when "001101" => controls <= "1010000010"; -- ORI
      when "000010" => controls <= "0000000100"; -- J
      when others   => controls <= "----------"; -- illegal op
    end case;
  end process;

  -- (regwrite, regdst, alusrc, branch, branch_ne, memwrite,
  --  memtoreg, jump, aluop(1 downto 0)) <= controls;
 (RegWrite, RegDst, ALUSrc, Branch, Branch_NE, MemWrite,
  MemToReg, Jump) <= controls(9 downto 2);
  ALUOp <= controls(1 downto 0);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity aludec is -- ALU control decoder
  port(funct:      in  STD_LOGIC_VECTOR(5 downto 0);
       aluop:      in  STD_LOGIC_VECTOR(1 downto 0);
       alucontrol: out STD_LOGIC_VECTOR(2 downto 0));
end;

architecture behave of aludec is
begin
  process(all) begin
    case aluop is
      when "00" => alucontrol <= "010"; -- add (for lw/sw/addi/j)
      when "01" => alucontrol <= "110"; -- sub (for beq/bne)
      when "10" => alucontrol <= "001"; -- or (for ori)
      when others => case funct is      -- R-type instructions
                         when "100000" => alucontrol <= "010"; -- add
                         when "100010" => alucontrol <= "110"; -- sub
                         when "100100" => alucontrol <= "000"; -- and
                         when "100101" => alucontrol <= "001"; -- or
                         when "101010" => alucontrol <= "111"; -- slt
                         when others   => alucontrol <= "---"; -- ???
                     end case;
    end case;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity datapath is  -- MIPS datapath
  port(clk, reset:        in  STD_LOGIC;
       memtoreg:          in  STD_LOGIC;
       alusrc, regdst:    in  STD_LOGIC;
       regwrite, jump:    in  STD_LOGIC;
       Branch, Branch_NE: in  STD_LOGIC;
       MemWrite:         in  STD_LOGIC;
       alucontrol:        in  STD_LOGIC_VECTOR(2 downto 0);
       zero:              out STD_LOGIC;
       pc:                buffer STD_LOGIC_VECTOR(31 downto 0);
       instr:             in  STD_LOGIC_VECTOR(31 downto 0);
       aluout, writedata: buffer STD_LOGIC_VECTOR(31 downto 0);
       readdata:          in  STD_LOGIC_VECTOR(31 downto 0));
end;

architecture struct of datapath is
  component alu
    port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
         alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
         result:     buffer STD_LOGIC_VECTOR(31 downto 0);
         zero:       out STD_LOGIC);
  end component;
  component regfile
    port(clk:           in  STD_LOGIC;
         we3:           in  STD_LOGIC;
         ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
         wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
         rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component adder
    port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
         y:    out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component sl2
    port(a: in  STD_LOGIC_VECTOR(31 downto 0);
         y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component signext
    port(a: in  STD_LOGIC_VECTOR(15 downto 0);
         y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;
  component flopr
    generic(width: integer);
    port(clk, reset: in  STD_LOGIC;
         en:         in  STD_LOGIC;
         d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
         q:          out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component mux2
    generic(width: integer);
    port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
         s:      in  STD_LOGIC;
         y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  component regaux1
    generic(W : integer);
    port (clk         : in std_logic;
          en          : in std_logic;

          RD          : in std_logic_vector(W-1 downto 0);
          
          PCPlus4F    : in std_logic_vector(W-1 downto 0);
          
          InstrD      : out std_logic_vector(W-1 downto 0);
          
          PCPlus4D    : out std_logic_vector(W-1 downto 0));
  end component;
  component regaux2
    generic(W : integer);
    port (clk         : in std_logic;
          en          : in std_logic;

          RegWriteD   : in std_logic;
          MemtoRegD   : in std_logic;
          MemWriteD   : in std_logic;
          BranchD     : in std_logic;
          Branch_NED  : in std_logic;
          ALUControlD : in std_logic_vector(2 downto 0);
          ALUSrcD     : in std_logic;
          RegDstD     : in std_logic;

          RD1         : in std_logic_vector(W-1 downto 0);
          RD2         : in std_logic_vector(W-1 downto 0);
          RsD         : in std_logic_vector(4 downto 0);
          RtD         : in std_logic_vector(4 downto 0);
          RdD         : in std_logic_vector(4 downto 0);
          SignImmD    : in std_logic_vector(W-1 downto 0);
          PCPlus4D    : in std_logic_vector(W-1 downto 0);

          RegWriteE   : out std_logic;
          MemtoRegE   : out std_logic;
          MemWriteE   : out std_logic;
          BranchE     : out std_logic;
          Branch_NEE  : out std_logic;
          ALUControlE : out std_logic_vector(2 downto 0);
          ALUSrcE     : out std_logic;
          RegDstE     : out std_logic;
  
          SrcAE       : out std_logic_vector(W-1 downto 0);
          WriteDataE  : out std_logic_vector(W-1 downto 0);
          RsE         : out std_logic_vector(4 downto 0);
          RtE         : out std_logic_vector(4 downto 0);
          RdE         : out std_logic_vector(4 downto 0);
          SignImmE    : out std_logic_vector(W-1 downto 0);
          PCPlus4E    : out std_logic_vector(W-1 downto 0));
end component;
component regaux3
  generic(W : integer);
  port (clk           : in std_logic;
        clr           : in std_logic;

        RegWriteE     : in std_logic;
        MemtoRegE     : in std_logic;
        MemWriteE     : in std_logic;
        BranchE       : in std_logic;
        Branch_NEE    : in std_logic;
        
        ZeroE         : in std_logic;
        ALUOutE       : in std_logic_vector(W-1 downto 0);
        WriteDataE    : in std_logic_vector(W-1 downto 0);
        WriteRegE     : in std_logic_vector(4 downto 0);
        PcBranchE     : in std_logic_vector(W-1 downto 0);
        
        RegWriteM     : out std_logic;
        MemtoRegM     : out std_logic;
        MemWriteM     : out std_logic;
        BranchM       : out std_logic;
        Branch_NEM    : out std_logic;
        
        ZeroM         : out std_logic;
        ALUOutM       : out std_logic_vector(W-1 downto 0);
        WriteDataM    : out std_logic_vector(W-1 downto 0);
        WriteRegM     : out std_logic_vector(4 downto 0);
        PcBranchM     : out std_logic_vector(W-1 downto 0));
end component;
component regaux4
  generic(W : integer);
  port (clk           : in std_logic;

        RegWriteM     : in std_logic;
        MemToRegM     : in std_logic;
        
        ReadDataM     : in std_logic_vector(W-1 downto 0);
        AluOutM       : in std_logic_vector(W-1 downto 0);
        WriteRegM     : in std_logic_vector(4 downto 0);
        
        RegWriteW     : out std_logic;
        MemToRegW     : out std_logic;
        
        ReadDataW     : out std_logic_vector(W-1 downto 0);
        AluOutW       : out std_logic_vector(W-1 downto 0);
        WriteRegW     : out std_logic_vector(4 downto 0));
end component;
component HazardUnit
  port (RsD:        in std_logic_vector(4 downto 0);
        RtD:        in std_logic_vector(4 downto 0);
        RsE:        in std_logic_vector(4 downto 0);
        RtE:        in std_logic_vector(4 downto 0);
        WriteRegM:  in std_logic_vector(4 downto 0);
        WriteRegW:  in std_logic_vector(4 downto 0);
        RegWriteM:  in std_logic;
        RegWriteW:  in std_logic;
        MemToRegE:  in std_logic;

        ForwardAE: out std_logic_vector(1 downto 0);
        ForwardBE: out std_logic_vector(1 downto 0);
        StallF:    out std_logic;
        StallD:    out std_logic;
        FlushE:    out std_logic);
end component;
component mux3 is -- three-input multiplexer
  generic(width: integer);
  port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:          in  STD_LOGIC_VECTOR(1 downto 0);
       
       y:          out STD_LOGIC_VECTOR(width-1 downto 0));
end component;

  signal pcjump, pcnext,
         pcnextbr, PCPlus4F,
         pcbranch, PCF:         STD_LOGIC_VECTOR(31 downto 0);
  signal StallF:                STD_LOGIC;

  signal SignImmD, RD1D, 
         RD2D, InstrD,
         PCPlus4D:              STD_LOGIC_VECTOR(31 downto 0);
  signal StallD:                STD_LOGIC;
  signal RsD, RtD, RdD:         STD_LOGIC_VECTOR(4 downto 0);

  signal RegWriteE, MemToRegE,
         MemWriteE, BranchE,
         Branch_NEE,
         ALUSrcE, RegDstE,
         ZeroE, FlushE:         STD_LOGIC;
  signal ALUControlE:           STD_LOGIC_VECTOR(2 downto 0);
  signal RD1E, SrcBE, ALUOutE,
         RD2E, SignImmE,
         SignImmShE, PCPlus4E,
         PCBranchE, ForwardedAE,
         ForwardedBE:           STD_LOGIC_VECTOR(31 downto 0);
  signal RsE, RtE, RdE,
         WriteRegE:             STD_LOGIC_VECTOR(4 downto 0);
  signal ForwardAE, ForwardBE:  STD_LOGIC_VECTOR(1 downto 0);

  signal ALUOutM, WriteDataM,
         ReadDataM, PCBranchM:  STD_LOGIC_VECTOR(31 downto 0);
  signal WriteRegM:             STD_LOGIC_VECTOR(4 downto 0);
  signal ZeroM, RegWriteM,
         MemToRegM, MemWriteM, 
         BranchM, Branch_NEM,
         PCSrcM:                STD_LOGIC;

  signal RegWriteW, MemToRegW:  STD_LOGIC;
  signal ALUOutW, ReadDataW,
         ResultW:               STD_LOGIC_VECTOR(31 downto 0);
  signal WriteRegW:             STD_LOGIC_VECTOR(4 downto 0);
begin

  -- IF

  pcbrmux: mux2 generic map(32) port map(PCPlus4F, PCBranchM, PCSrcM, pcnextbr);
  pcjump <= PCPlus4D(31 downto 28) & InstrD(25 downto 0) & "00";
  pcmux: mux2 generic map(32) port map(pcnextbr, pcjump, jump, pcnext);
  pcreg: flopr generic map(32) port map(clk, reset, StallF, pcnext, PCF);
  pc <= PCF;
  pcadd1: adder port map(PCF, X"00000004", PCPlus4F);

  regpipe1: regaux1 generic map(32) port map(clk, StallD, instr, PCPlus4F, InstrD, PCPlus4D);
 
  -- ID

  RsD <= InstrD(25 downto 21);
  RtD <= InstrD(20 downto 16);
  RdD <= InstrD(15 downto 11);
  
  rf: regfile port map(
    clk, regwrite, RsD, RtD, WriteRegW, ResultW,
    RD1D, RD2D
  );
  se: signext port map(InstrD(15 downto 0), SignImmD);

  regpipe2: regaux2 generic map(32) port map(clk, StallD,
    regwrite, memtoreg, memwrite, Branch, Branch_NE, alucontrol, alusrc, regdst,
    RD1D, RD2D, RsD, RtD, RdD, SignImmD, PCPlus4D,
    RegWriteE, MemToRegE, MemWriteE, BranchE, Branch_NEE, ALUControlE, ALUSrcE, RegDstE,
    RD1E, RD2E, RsE, RtE, RdE, SignImmE, PCPlus4E
  );

  -- EX

  fwdamux: mux3 generic map(32) port map(RD1E, ResultW, ALUOutM, ForwardAE, ForwardedAE);
  fwdbmux: mux3 generic map(32) port map(RD2E, ResultW, ALUOutM, ForwardBE, ForwardedBE);

  srcbmux: mux2 generic map(32) port map(ForwardedBE, SignImmE, alusrc, SrcBE);
  mainalu: alu port map(ForwardedAE, SrcBE, ALUControlE, ALUOutE, ZeroE);

  wrmux: mux2 generic map(5) port map(RtE, RdE, RegDstE, WriteRegE);

  immsh: sl2 port map(SignImmD, SignImmShE);
  pcadd2: adder port map(PCPlus4D, SignImmShE, PCBranchE);

  regpipe3: regaux3 generic map(32) port map(clk, FlushE,
    RegWriteE, MemToRegE, MemWriteE, BranchE, Branch_NEE,
    ZeroE, ALUOutE, ForwardedBE, WriteRegE, PCBranchE,
    RegWriteM, MemToRegM, MemWriteM, BranchM, Branch_NEM,
    ZeroM, ALUOutM, WriteDataM, WriteRegM, PCBranchM
  );

  -- MEM

  PCSrcM <= ((BranchM and ZeroM) or (Branch_NEM and (not ZeroM)));

  regpipe4: regaux4 generic map(32) port map(clk,
    RegWriteM, MemToRegM,
    ALUOutM, ReadDataM, WriteRegM,
    RegWriteW, MemToRegW,
    ALUOutW, ReadDataW, WriteRegW
  );

  -- WB

  resmux: mux2 generic map(32) port map(ALUOutW, ReadDataW, MemToRegW, ResultW);

  -- HU

  hu: HazardUnit port map(
    RsD, RtD, RsE, RtE, WriteRegM, WriteRegW, RegWriteM, RegWriteW, MemToRegE,
    ForwardAE, ForwardBE, StallF, StallD, FlushE
  );
 
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity regfile is -- three-port register file
  port(clk:           in  STD_LOGIC;
       we3:           in  STD_LOGIC;
       ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0);
       wd3:           in  STD_LOGIC_VECTOR(31 downto 0);
       rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of regfile is
  type ramtype is array (31 downto 0) of STD_LOGIC_VECTOR(31 downto 0);
  signal mem: ramtype;
begin
  -- three-ported register file
  -- read two ports combinationally
  -- write third port on rising edge of clock
  -- register 0 hardwired to 0
  -- note: for pipelined processor, write third port
  -- on falling edge of clk
  process(clk) begin
    if rising_edge(clk) then
       if we3 = '1' then mem(to_integer(wa3)) <= wd3;
       end if;
    end if;
  end process;
  process(all) begin
    if (to_integer(ra1) = 0) then rd1 <= X"00000000"; -- register 0 holds 0
    else rd1 <= mem(to_integer(ra1));
    end if;
    if (to_integer(ra2) = 0) then rd2 <= X"00000000";
    else rd2 <= mem(to_integer(ra2));
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity adder is -- adder
  port(a, b: in  STD_LOGIC_VECTOR(31 downto 0);
       y:    out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of adder is
begin
  y <= a + b;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;

entity sl2 is -- shift left by 2
  port(a: in  STD_LOGIC_VECTOR(31 downto 0);
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of sl2 is
begin
  y <= a(29 downto 0) & "00";
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity signext is -- sign extender
  port(a: in  STD_LOGIC_VECTOR(15 downto 0);
       y: out STD_LOGIC_VECTOR(31 downto 0));
end;

architecture behave of signext is
begin
  y <= X"ffff" & a when a(15) else X"0000" & a;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;  use IEEE.STD_LOGIC_ARITH.all;

entity flopr is -- flip-flop with enable and synchronous reset
  generic(width: integer);
  port(clk, reset: in  STD_LOGIC;
       en:         in  STD_LOGIC;
       d:          in  STD_LOGIC_VECTOR(width-1 downto 0);
       q:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture asynchronous of flopr is
begin
  process(all) begin
    if reset then  q <= (others => '0');
    elsif (rising_edge(clk) and en = '0') then
      q <= d;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mux2 is -- two-input multiplexer
  generic(width: integer);
  port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:      in  STD_LOGIC;
       y:      out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux2 is
begin
  y <= d1 when s else d0;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity alu is
  port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
       alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
       result:     buffer STD_LOGIC_VECTOR(31 downto 0);
       zero:       out STD_LOGIC);
end;

architecture behave of alu is
  signal condinvb, sum: STD_LOGIC_VECTOR(31 downto 0);
begin
  condinvb <= not b when alucontrol(2) else b;
  sum <= a + condinvb + alucontrol(2);

  process(all) begin
    case alucontrol(1 downto 0) is
      when "00"   => result <= a and b;
      when "01"   => result <= a or b;
      when "10"   => result <= sum;
      when "11"   => result <= (0 => sum(31), others => '0');
      when others => result <= (others => 'X');
    end case;
  end process;

  zero <= '1' when result = X"00000000" else '0';
end;


-- Auxiliary register 1 - IF stage to ID stage
library ieee;
use ieee.std_logic_1164.all;

Entity regaux1 is
  Generic(W : integer);
  Port (clk         : in std_logic;
        en          : in std_logic;

        RD          : in std_logic_vector(W-1 downto 0);

        PCPlus4F    : in std_logic_vector(W-1 downto 0);
        
        InstrD      : out std_logic_vector(W-1 downto 0);
        
        PCPlus4D    : out std_logic_vector(W-1 downto 0));
  End;

Architecture behave of regaux1 is
begin
  process(all)
  begin
    if(clk'event and clk = '1' and en = '0') then
      InstrD <= RD;
      
      PCPlus4D <= PCPlus4F;
    end if;
  end process;
end;


-- Auxiliary register 2 - ID stage to EX stage
library ieee;
use ieee.std_logic_1164.all;

Entity regaux2 is
  Generic(W : integer);
  Port (clk         : in std_logic;
        en          : in std_logic;

        RegWriteD   : in std_logic;
        MemtoRegD   : in std_logic;
        MemWriteD   : in std_logic;
        BranchD     : in std_logic;
        Branch_NED  : in std_logic;
        ALUControlD : in std_logic_vector(2 downto 0);
        ALUSrcD     : in std_logic;
        RegDstD     : in std_logic;

        RD1         : in std_logic_vector(W-1 downto 0);
        RD2         : in std_logic_vector(W-1 downto 0);
        RsD         : in std_logic_vector(4 downto 0);
        RtD         : in std_logic_vector(4 downto 0);
        RdD         : in std_logic_vector(4 downto 0);
        SignImmD    : in std_logic_vector(W-1 downto 0);
        PCPlus4D    : in std_logic_vector(W-1 downto 0);
        
        RegWriteE   : out std_logic;
        MemtoRegE   : out std_logic;
        MemWriteE   : out std_logic;
        BranchE     : out std_logic;
        Branch_NEE  : out std_logic;
        ALUControlE : out std_logic_vector(2 downto 0);
        ALUSrcE     : out std_logic;
        RegDstE     : out std_logic;

        SrcAE       : out std_logic_vector(W-1 downto 0);
        WriteDataE  : out std_logic_vector(W-1 downto 0);
        RsE         : out std_logic_vector(4 downto 0);
        RtE         : out std_logic_vector(4 downto 0);
        RdE         : out std_logic_vector(4 downto 0);
        SignImmE    : out std_logic_vector(W-1 downto 0);
        PCPlus4E    : out std_logic_vector(W-1 downto 0));
End;

Architecture behave of regaux2 is
begin
  process(all)
  begin
    if(clk'event and clk = '1') then
      RegWriteE   <= RegWriteD;
      MemtoRegE   <= MemtoRegD;
      MemWriteE   <= MemWriteD;
      BranchE     <= BranchD;
      Branch_NEE  <= Branch_NED;
      ALUControlE <= ALUControlD;
      ALUSrcE     <= ALUSrcD;
      RegDstE     <= RegDstD;

      SrcAE      <= RD1;
      WriteDataE <= RD2;
      RsE        <= RsD;
      RtE        <= RtD;
      RdE        <= RdD;
      SignImmE   <= SignImmD;
      PCPlus4E   <= PCPlus4D;
    end if;
  end process;
end;


-- Auxiliary register 3 - EX stage to MEM stage
library ieee;
use ieee.std_logic_1164.all;

Entity regaux3 is
  Generic(W : integer);
  Port (clk           : in std_logic;
        clr           : in std_logic;

        RegWriteE     : in std_logic;
        MemtoRegE     : in std_logic;
        MemWriteE     : in std_logic;
        BranchE       : in std_logic;
        Branch_NEE    : in std_logic;
        
        ZeroE         : in std_logic;
        ALUOutE       : in std_logic_vector(W-1 downto 0);
        WriteDataE    : in std_logic_vector(W-1 downto 0);
        WriteRegE     : in std_logic_vector(4 downto 0);
        PcBranchE     : in std_logic_vector(W-1 downto 0);
        
        RegWriteM     : out std_logic;
        MemtoRegM     : out std_logic;
        MemWriteM     : out std_logic;
        BranchM       : out std_logic;
        Branch_NEM    : out std_logic;
        
        ZeroM         : out std_logic;
        ALUOutM       : out std_logic_vector(W-1 downto 0);
        WriteDataM    : out std_logic_vector(W-1 downto 0);
        WriteRegM     : out std_logic_vector(4 downto 0);
        PcBranchM     : out std_logic_vector(W-1 downto 0));
end;

Architecture behave of regaux3 is
begin
  process(all)
  begin
    if(clk'event and clk = '1') then
      RegWriteM  <= RegWriteE;
      MemtoRegM  <= MemtoRegE;
      MemWriteM  <= MemWriteE;
      BranchM    <= BranchE;
      Branch_NEM <= Branch_NEE;
      
      ZeroM      <= ZeroE;
      AluOutM    <= AluOutE;
      WriteDataM <= WriteDataE;
      WriteRegM  <= WriteRegE;
      PcBranchM  <= PcBranchE;
    end if;
  end process;
end;


-- Auxiliary register 4 - MEM stage to WB stage
library ieee;
use ieee.std_logic_1164.all;

Entity regaux4 is
  Generic(W : integer);
  Port (clk           : in std_logic;
  
        RegWriteM     : in std_logic;
        MemToRegM     : in std_logic;
        
        ReadDataM     : in std_logic_vector(W-1 downto 0);
        AluOutM       : in std_logic_vector(W-1 downto 0);
        WriteRegM     : in std_logic_vector(4 downto 0);
        
        RegWriteW     : out std_logic;
        MemToRegW     : out std_logic;
        
        ReadDataW     : out std_logic_vector(W-1 downto 0);
        AluOutW       : out std_logic_vector(W-1 downto 0);
        WriteRegW     : out std_logic_vector(4 downto 0));
End;

Architecture behave of regaux4 is
begin
  process(all)
  begin
    if(clk'event and clk = '1') then
      RegWriteW <= RegWriteM;
      MemToRegW <= MemToRegM;

      AluOutW   <= AluOutM;
      ReadDataW <= ReadDataM;
      WriteRegW <= WriteRegM;
    end if;
  end process;
end;


library ieee;
use ieee.std_logic_1164.all;

entity HazardUnit is
  port (RsD:        in std_logic_vector(4 downto 0);
        RtD:        in std_logic_vector(4 downto 0);
        RsE:        in std_logic_vector(4 downto 0);
        RtE:        in std_logic_vector(4 downto 0);
        WriteRegM:  in std_logic_vector(4 downto 0);
        WriteRegW:  in std_logic_vector(4 downto 0);
        RegWriteM:  in std_logic;
        RegWriteW:  in std_logic;
        MemToRegE:  in std_logic;

        ForwardAE: out std_logic_vector(1 downto 0);
        ForwardBE: out std_logic_vector(1 downto 0);
        StallF:    out std_logic;
        StallD:    out std_logic;
        FlushE:    out std_logic
  );
end;

architecture behave of HazardUnit is
begin
  -- data hazard: forwarding
  process(RsE, RtE, WriteRegW, WriteRegM)
  begin
    if((RsE /= "0000") and (RsE = WriteRegM) and (RegWriteM = '1')) then
      ForwardAE <= "10";
    elsif((RsE /= "0000") and (RsE = WriteRegM) and (RegWriteW = '1')) then
      ForwardAE <= "01";
    else
      ForwardAE <= "00";
    end if;
  end process;

  -- data hazard: stalling
  process(RsD, RtD, RtE, MemToRegE)
  begin
    if ((((RsD = RtE) or (RtD = RtE)) and (MemToRegE = '1'))) then
      StallF <= '1';
      StallD <= '1';
      FlushE <= '1';
    else
      StallF <= '0';
      StallD <= '0';
      FlushE <= '0';
    end if;
  end process;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mux3 is -- three-input multiplexer
  generic(width: integer);
  port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:          in  STD_LOGIC_VECTOR(1 downto 0);
       
       y:          out STD_LOGIC_VECTOR(width-1 downto 0));
end;

architecture behave of mux3 is
begin
  with s select
    y <= d0 when "00",
         d1 when "01",
         d2 when others;
end;
