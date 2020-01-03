-- mips.vhd
-- From Section 7.6 of Digital Design & Computer Architecture
-- Updated to VHDL 2008 26 July 2011 David_Harris@hmc.edu

library IEEE;
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;

entity testbench is
end;

architecture test of testbench is
  component top
    port(clk, reset:           in  std_logic;
         writedata, dataadr:   out std_logic_vector(31 downto 0);
         memwrite:             out std_logic);
  end component;
  signal writedata, dataadr:    std_logic_vector(31 downto 0);
  signal clk, reset,  memwrite: std_logic;
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
    if (falling_edge(clk) and memwrite = '1') then
    --if to_integer(dataadr) = 84 then
      if (to_integer(dataadr) = 84 and to_integer(writedata) = 7) then
      --if (to_integer(dataadr) = 84 and writedata = "11111111111111110111111100000010") then
        report "NO ERRORS: Simulation succeeded" severity failure;
      elsif (dataadr /= 80) then
        report "Simulation failed, dataadr = " & integer'image(to_integer(dataadr)) severity failure;
      end if;
    end if;
  end process;
end;

library IEEE;
use IEEE.STD_LOGIC_1164.all; use IEEE.NUMERIC_STD_UNSIGNED.all;

entity top is -- top-level design for testing
  port(clk, reset:           in     std_logic;
       writedata, dataadr:   buffer std_logic_vector(31 downto 0);
       memwrite:             buffer std_logic);
end;

architecture test of top is
  component mips
    port(clk, reset:        in  std_logic;
         pc:                out std_logic_vector(31 downto 0);
         instr:             in  std_logic_vector(31 downto 0);
         memwrite:          out std_logic;
         aluout, writedata: out std_logic_vector(31 downto 0);
         readdata:          in  std_logic_vector(31 downto 0));
  end component;
  component imem
    port(a:  in  std_logic_vector(5 downto 0);
         rd: out std_logic_vector(31 downto 0));
  end component;
  component dmem
    port(clk, we:  in std_logic;
         a, wd:    in std_logic_vector(31 downto 0);
         rd:       out std_logic_vector(31 downto 0));
  end component;
  signal pc, instr,
         readdata: std_logic_vector(31 downto 0);
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
  port(clk, we:  in std_logic;
       a, wd:    in std_logic_vector(31 downto 0);
       rd:       out std_logic_vector(31 downto 0));
end;

architecture behave of dmem is
begin
   process is
     type ramtype is array (63 downto 0) of std_logic_vector(31 downto 0);
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
  port(a:  in  std_logic_vector(5 downto 0);
       rd: out std_logic_vector(31 downto 0));
end;

architecture behave of imem is
begin
   process is
     file mem_file: TEXT;
     variable L: line;
     variable ch: character;
     variable i, index, result: integer;
     type ramtype is array (63 downto 0) of std_logic_vector(31 downto 0);
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

entity mips is -- pipelined MIPS processor
  port(clk, reset:        in  std_logic;
       pc:                out std_logic_vector(31 downto 0);
       instr:             in  std_logic_vector(31 downto 0);
       memwrite:          out std_logic;
       aluout, writedata: out std_logic_vector(31 downto 0);
       readdata:          in  std_logic_vector(31 downto 0));
end;

architecture struct of mips is

  component controller
    port(op, funct:          in  std_logic_vector(5 downto 0);
         zero:               in  std_logic;
         memtoreg, memwrite: out std_logic;
         pcsrc, alusrc:      out std_logic;
         regdst, regwrite:   out std_logic;
         jump:               out std_logic;
		 branch, branchn:  out std_logic;
         alucontrol:         out std_logic_vector(2 downto 0));
  end component;
  
  component datapath
    port(clk, reset:        in  std_logic;
         memtoreg, pcsrc:   in  std_logic;
         alusrc, regdst:    in  std_logic;
         regwrite, jump:    in  std_logic;
         alucontrol:        in  std_logic_vector(2 downto 0);
         memwrite:          in  std_logic;
         zero:              out std_logic;
         pc:                buffer std_logic_vector(31 downto 0);
         instr:             in std_logic_vector(31 downto 0);
         aluout, writedata: buffer std_logic_vector(31 downto 0);
         readdata:          in  std_logic_vector(31 downto 0);
		 branch, branchn: in  STD_LOGIC);
  end component;
  
  signal memtoreg, alusrc, regdst, regwrite, jump, pcsrc: std_logic;
  signal zero, s_branch, s_branchn: std_logic;
  signal alucontrol: std_logic_vector(2 downto 0);
begin
  cont: controller port map(instr(31 downto 26), instr(5 downto 0),
                            zero, memtoreg, memwrite, pcsrc, alusrc,
                            regdst, regwrite, jump, s_branch, s_branchn, alucontrol);
  dp: datapath port map(clk, reset, memtoreg, pcsrc, alusrc, regdst,
                        regwrite, jump, alucontrol, memwrite, zero, pc, instr,
                        aluout, writedata, readdata, s_branch, s_branchn);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity controller is -- single cycle control decoder
  port(op, funct:          in  std_logic_vector(5 downto 0);
       zero:               in  std_logic;
       memtoreg, memwrite: out std_logic;
       pcsrc, alusrc:      out std_logic;
       regdst, regwrite:   out std_logic;
       jump:               out std_logic;
	   branch, branchn:  out std_logic;
       alucontrol:         out std_logic_vector(2 downto 0));
end;

architecture struct of controller is
  component maindec
    port(op:                 in  std_logic_vector(5 downto 0);
         memtoreg, memwrite: out std_logic;
         branch, branchn:  out std_logic;
		 alusrc: 			 out std_logic;
         regdst, regwrite:   out std_logic;
         jump:               out std_logic;
         aluop:              out std_logic_vector(1 downto 0));
  end component;
  component aludec
    port(funct:      in  std_logic_vector(5 downto 0);
         aluop:      in  std_logic_vector(1 downto 0);
         alucontrol: out std_logic_vector(2 downto 0));
  end component;
  signal aluop:  std_logic_vector(1 downto 0);
  signal s_branch, s_branchn: std_logic;
  
begin
  md: maindec port map(op, memtoreg, memwrite, s_branch, s_branchn,
                       alusrc, regdst, regwrite, jump, aluop);
  ad: aludec port map(funct, aluop, alucontrol);

  pcsrc <= (branch and zero) or (branchn and not zero);
  branch <= s_branch;
  branchn <= s_branchn;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;

entity maindec is -- main control decoder
  port(op:                 in  std_logic_vector(5 downto 0);
       memtoreg, memwrite: out std_logic;
       branch, branchn:  out std_logic;
	   alusrc:    		   out std_logic;
       regdst, regwrite:   out std_logic;
       jump:               out std_logic;
       aluop:              out std_logic_vector(1 downto 0));
end;

architecture behave of maindec is
  signal controls: std_logic_vector(9 downto 0);
begin
  process(all) begin
    case op is
      when "000000" => controls <= "0110000010"; -- RTYPE
      when "100011" => controls <= "0101001000"; -- LW
      when "101011" => controls <= "0001010000"; -- SW
      when "000100" => controls <= "0000100001"; -- BEQ
      when "001000" => controls <= "0101000000"; -- ADDI
      when "000010" => controls <= "0000000100"; -- J
	  when "001101" => controls <= "0101000011"; -- ORI
	  when "000101" => controls <= "1000000001"; -- BNE
      when others   => controls <= "----------"; -- illegal op
    end case;
  end process;

  -- (regwrite, regdst, alusrc, branch, memwrite,
  --  memtoreg, jump, aluop(1 downto 0)) <= controls;
 (branchn, regwrite, regdst, alusrc, branch, memwrite,
  memtoreg, jump) <= controls(9 downto 2);
  aluop <= controls(1 downto 0);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity aludec is -- ALU control decoder
  port(funct:      in  std_logic_vector(5 downto 0);
       aluop:      in  std_logic_vector(1 downto 0);
       alucontrol: out std_logic_vector(2 downto 0));
end;

architecture behave of aludec is
begin
  process(all) begin
    case aluop is
      when "00" => alucontrol <= "010"; -- add (for lw/sw/addi)
      when "01" => alucontrol <= "110"; -- sub (for beq)
	  when "11" => alucontrol <= "001"; -- or (for ori)
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
  port(clk, reset:        in  std_logic;
      memtoreg, pcsrc:    in  std_logic;
      alusrc, regdst:     in  std_logic;
      regwrite, jump:     in  std_logic;
      alucontrol:         in  std_logic_vector(2 downto 0);
      memwrite:           in  std_logic;
      zero:               out std_logic;
      pc:                 buffer std_logic_vector(31 downto 0);
      instr:              in  std_logic_vector(31 downto 0);
      aluout, writedata:  buffer std_logic_vector(31 downto 0);
      readdata:           in  std_logic_vector(31 downto 0);
	  branch, branchn:  in std_logic);
end;

architecture struct of datapath is

  component fetch
    port (
      clock, reset:   in std_logic;
      PCsrcM:         in std_logic;
      PCbranchM:		  in std_logic_vector(31 downto 0);
      stallF:         in std_logic;
      PCplus4F:       out std_logic_vector(31 downto 0);
      instrF:         out std_logic_vector(31 downto 0)
    );
  end component;

  component regaux1
    port(
        clock, clear:         in std_logic; 
        stallD:	              in std_logic;
        PCplus4F, instrF:	  	in std_logic_vector(31 downto 0);
        PCplus4D, instrD:		  out std_logic_vector(31 downto 0));
  end component;

  component decode
    port (
      -- entradas
      clock, reset:       in std_logic;
      regWriteW:          in std_logic;
      writeRegW:          in std_logic_vector(4 downto 0);
      instrD, PCplus4Din: in std_logic_vector(31 downto 0);
      resultW:            in std_logic_vector(31 downto 0);
      -- saidas
      PCplus4Dout:        out std_logic_vector(31 downto 0);
      RD1, RD2:           out std_logic_vector(31 downto 0);
      signImmD:           out std_logic_vector(31 downto 0);
      opcode, funct:      out std_logic_vector(5 downto 0);
      rsD, rtD, rdD:      out std_logic_vector(4 downto 0)
    );
  end component;  

  component regaux2
    port (
      clock, flushE: in std_logic;
      -- fluxo de dados:
        -- entradas
        PCplus4D:       in std_logic_vector(31 downto 0);
        signImmD:       in std_logic_vector(31 downto 0); 
        rd1D, rd2D:     in std_logic_vector(31 downto 0);
        rsD, rtD, rdD:  in std_logic_vector(4 downto 0);
        -- saidas
        PCplus4E:       out std_logic_vector(31 downto 0);
        signImmE:       out std_logic_vector(31 downto 0); 
        rd1E, rd2E:     out std_logic_vector(31 downto 0);
        rsE, rtE, rdE:  out std_logic_vector(4 downto 0);
  
      -- unidade de controle:
        -- entradas
        regWriteD:      in std_logic;
        memToRegD: 	    in std_logic;
        memWriteD:      in std_logic; 
        branchD:        in std_logic;
        ALUControlD:		in std_logic_vector(2 downto 0);
        ALUSrcD: 		    in std_logic;
        regDstD: 				in std_logic;
        -- saidas
        regWriteE:      out std_logic;
        memToRegE: 	    out std_logic;
        memWriteE:      out std_logic; 
        branchE:        out std_logic;
        ALUControlE:		out std_logic_vector(2 downto 0);
        ALUSrcE: 		    out std_logic;
        regDstE: 				out std_logic
    );
  end component;

  component execute
    port (
      clock:            in std_logic;
      -- entradas
        -- fluxo de dados:
        rtE, rdE:             in std_logic_vector(4 downto 0);
        rd1E, rd2E:           in std_logic_vector(31 downto 0);
        signImmE:             in std_logic_vector(31 downto 0);
        PCplus4E:             in std_logic_vector(31 downto 0);
        resultW:              in std_logic_vector(31 downto 0);
        ALUOutM:              in std_logic_vector(31 downto 0);
        forwardAE, forwardBE: in std_logic_vector(1 downto 0);
        -- unidade de controle:
        regDstE:              in std_logic;
        ALUSrcE:              in std_logic;
        ALUControlE:          in std_logic_vector(2 downto 0);
      -- saidas
        zeroE:                out std_logic;
        ALUOutE:              out std_logic_vector(31 downto 0);
        writeDataE:           out std_logic_vector(31 downto 0);
        writeRegE:            out std_logic_vector(4 downto 0);
        PCbranchE:            out std_logic_vector(31 downto 0)
    );
  end component;

  component regaux3
    port (
    clock, clear:            in std_logic;
    -- fluxo de dados
      -- entradas
      zeroE:          in std_logic;
      ALUoutE:        in std_logic_vector(31 downto 0);
      writeDataE:     in std_logic_vector(31 downto 0);
      writeRegE:      in std_logic_vector(4 downto 0);
      PCbranchE:      in std_logic_vector(31 downto 0);
      -- saidas
      zeroM:          out std_logic;
      ALUoutM:        out std_logic_vector(31 downto 0);
      writeDataM:     out std_logic_vector(31 downto 0);
      writeRegM:      out std_logic_vector(4 downto 0);
      PCbranchM:      out std_logic_vector(31 downto 0);
    -- unidade de controle
      -- entradas
      regWriteE:      in std_logic;
      memToRegE: 	    in std_logic;
      memWriteE:      in std_logic; 
      branchE:        in std_logic;
      -- saidas
      regWriteM:      out std_logic;
      memToRegM: 	    out std_logic;
      memWriteM:      out std_logic; 
      branchM:        out std_logic
    );
  end component;

  component memory
    port (
      clock:            in std_logic;
      -- entradas
        -- fluxo de dados:
        ALUOutM:        in std_logic_vector(31 downto 0);
        writeDataM:     in std_logic_vector(31 downto 0);
        writeRegM:      in std_logic_vector(4 downto 0);
        PCbranchM:      in std_logic_vector(31 downto 0);
        -- unidade de controle:
        memWriteM:      in std_logic;
      -- saidas
        ALUOutW:        out std_logic_vector(31 downto 0);
        rdM:            out std_logic_vector(31 downto 0);
        writeRegW:      out std_logic_vector(4 downto 0); 
        PCbranchF:      out std_logic_vector(31 downto 0)
    );
  end component;

  component regaux4
    port (
    clock, clear:            in std_logic;
    -- fluxo de dados
      -- entradas
      ALUOutM:        in std_logic_vector(31 downto 0);
      rdM:            in std_logic_vector(31 downto 0);
      writeRegM:      in std_logic_vector(4 downto 0); 
      -- saidas
      ALUOutW:        out std_logic_vector(31 downto 0);
      readDataW:      out std_logic_vector(31 downto 0);
      writeRegW:      out std_logic_vector(4 downto 0); 
    -- unidade de controle
      -- entradas
      regWriteM:      in std_logic;
      memToRegM: 	    in std_logic;
      -- saidas
      regWriteW:      out std_logic;
      memToRegW: 	    out std_logic
    );
  end component;

  component writeback
    port (
      clock:            in std_logic;
      -- entradas
        -- fluxo de dados:
        ALUOutW:        in std_logic_vector(31 downto 0);
        readDataW:      in std_logic_vector(31 downto 0);
        writeRegW_in:   in std_logic_vector(4 downto 0);       
        -- unidade de controle:
        memToRegW: 	    in std_logic;
      -- saidas
        resultW:        out std_logic_vector(31 downto 0);
        writeRegW_out:  out std_logic_vector(4 downto 0)
    );
  end component;

  component forwarding
    port (
      -- entradas
      rsE, rtE:               in std_logic_vector (4 downto 0);
      writeRegM, writeRegW:   in std_logic_vector (4 downto 0);
      regWriteM, regWriteW:   in std_logic;
      -- saidas
      forwardAE, forwardBE:   out std_logic_vector (1 downto 0)
    );
  end component;

  component stall_unit
    port (
      -- entradas
      rsD, rtD, rtE:            in std_logic_vector (4 downto 0);
      memToRegE:                in std_logic;
      -- saidas
      stallF, stallD, flushE:   out std_logic
    );
  end component;

  component jump_unit is
    port (
      register1, register2: in STD_LOGIC_VECTOR(31 downto 0);
      equal : out STD_LOGIC);
  end component jump_unit;

  component alu
    port(a, b:       in  std_logic_vector(31 downto 0);
         alucontrol: in  std_logic_vector(2 downto 0);
         result:     buffer std_logic_vector(31 downto 0);
         zero:       out std_logic);
  end component;
  component regfile
    port(clk:           in  std_logic;
         we3:           in  std_logic;
         ra1, ra2, wa3: in  std_logic_vector(4 downto 0);
         wd3:           in  std_logic_vector(31 downto 0);
         rd1, rd2:      out std_logic_vector(31 downto 0));
  end component;
  component adder
    port(a, b: in  std_logic_vector(31 downto 0);
         y:    out std_logic_vector(31 downto 0));
  end component;
  component sl2
    port(a: in  std_logic_vector(31 downto 0);
         y: out std_logic_vector(31 downto 0));
  end component;
  component signext
    port(a: in  std_logic_vector(15 downto 0);
         y: out std_logic_vector(31 downto 0));
  end component;
  component flopr generic(width: integer);
    port(clk, reset: in  std_logic;
         d:          in  std_logic_vector(width-1 downto 0);
         q:          out std_logic_vector(width-1 downto 0));
  end component;
  component mux2 generic(width: integer);
    port(d0, d1: in  std_logic_vector(width-1 downto 0);
         s:      in  std_logic;
         y:      out std_logic_vector(width-1 downto 0));
  end component;
  signal writereg:           std_logic_vector(4 downto 0);
  signal pcjump, pcnext,
         pcnextbr, pcplus4,
         pcbranch:           std_logic_vector(31 downto 0);
  signal signimm, signimmsh: std_logic_vector(31 downto 0);
  signal srca, srcb, result: std_logic_vector(31 downto 0);

  signal s_PCbranchM, s_PCplus4F, s_instrF: std_logic_vector(31 downto 0);
  signal s_PCplus4D, s_instrD:              std_logic_vector(31 downto 0);
  signal s_writeRegW:                       std_logic_vector(4 downto 0);
  signal s_resultW:                         std_logic_vector(31 downto 0);
  signal s_PCplus4Din, s_PCplus4Dout:       std_logic_vector(31 downto 0);
  signal s_rd1D, s_rd2D, s_signImmD:        std_logic_vector(31 downto 0);
  signal s_rsD, s_rtD, s_rdD:               std_logic_vector(4 downto 0);
  signal s_rd1E, s_rd2E, s_signImmE:        std_logic_vector(31 downto 0);
  signal s_rsE, s_rtE, s_rdE:               std_logic_vector(4 downto 0);
  signal opcode, funct:                     std_logic_vector(5 downto 0);
  signal s_PCplus4E:                        std_logic_vector(31 downto 0);
  signal s_ALUControlE:                     std_logic_vector(2 downto 0);
  signal s_regWriteE, s_memToRegE, 
         s_memWriteE, s_branchE, s_ALUSrcE, 
         s_regDstE:                         std_logic;
  signal s_ALUOutE, s_writeDataE:           std_logic_vector(31 downto 0); 
  signal s_PCbranchE:                       std_logic_vector(31 downto 0);
  signal s_writeRegE:                       std_logic_vector(4 downto 0);
  signal s_zeroE:                           std_logic;
  signal s_ALUoutM, s_writeDataM:           std_logic_vector(31 downto 0);
  signal s_PCbranchE_out:                   std_logic_vector(31 downto 0);
  signal s_writeRegM:                       std_logic_vector(4 downto 0);
  signal s_regWriteM, s_memToRegM, 
         s_memWriteM, s_branchM:            std_logic;
  signal s_ALUOutMout, s_rdM:               std_logic_vector(31 downto 0);
  signal s_writeRegMout:                    std_logic_vector(4 downto 0);
  signal s_ALUOutW, s_readDataW:            std_logic_vector(31 downto 0);
  signal s_writeRegW_out:                   std_logic_vector(4 downto 0);
  signal s_regWriteW, s_memToRegW:          std_logic;
  signal s_forwardAE, s_forwardBE:          std_logic_vector(1 downto 0);
  signal s_rsEout, s_rtEout:                std_logic_vector(4 downto 0);
  signal s_stallF, s_stallD, s_flushE:      std_logic;
begin
  
  instr_f: fetch port map (
    clk, reset, pcsrc, s_PCbranchM, s_stallF, -- entradas
    s_PCplus4F, s_instrF            -- saidas
  );

  ifid: regaux1 port map (
    clk, pcsrc, s_stallD, s_PCplus4F, s_instrF,   -- entradas
    s_PCplus4Din, s_instrD                   -- saidas
  );

  id: decode port map (
    clk, reset, s_regWriteW, s_writeRegW_out, s_instrD, s_PCplus4Din, s_resultW,      -- entradas
    s_PCplus4Dout, s_rd1D, s_rd2D, s_signImmD, opcode, funct, s_rsD, s_rtD, s_rdD -- saidas
  );

  idex: regaux2 port map (
    -- fluxo de dados
    clk, s_flushE, s_PCplus4Dout, s_signImmD, s_rd1D, s_rd2D, s_rsD, s_rtD, s_rdD, -- entradas
    s_PCplus4E, s_signImmE, s_rd1E, s_rd2E, s_rsE, s_rtE, s_rdE,                -- saidas
    -- unidade de controle
    regwrite, memtoreg, memwrite, '0', alucontrol, alusrc, regdst,                        -- entradas
    s_regWriteE, s_memToRegE, s_memWriteE, s_branchE, s_ALUControlE, s_ALUSrcE, s_regDstE -- saidas
  );

  ex: execute port map (
    clk, s_rtE, s_rdE, s_rd1E, s_rd2E, s_signImmE, s_PCplus4E, s_resultW, s_ALUOutM, s_forwardAE, s_forwardBE, s_regDstE, s_ALUSrcE, s_ALUControlE, -- entradas
    s_zeroE, s_ALUOutE, s_writeDataE, s_writeRegE, s_PCbranchE -- saidas 

  );

  exmem: regaux3 port map (
    -- fluxo de dados
    clk, '0', s_zeroE, s_ALUOutE, s_writeDataE, s_writeRegE, s_PCbranchE, -- entradas
    open, s_ALUOutM, s_writeDataM, s_writeRegM, s_PCbranchE_out,              -- saidas
    -- unidade de controle
    s_regWriteE, s_memToRegE, s_memWriteE, s_branchE, -- entradas
    s_regWriteM, s_memToRegM, s_memWriteM, s_branchM  -- saidas
  );

  mem: memory port map (
    clk, s_ALUOutMout, s_writeDataM, s_writeRegM, s_PCbranchE_out, s_memWriteM, -- entradas
    s_ALUOutW, s_rdM, s_writeRegMout, s_PCbranchM                           -- saidas
  );

  memwb: regaux4 port map (
    -- fluxo de dados
    clk, '0', s_ALUOutMout, s_rdM, s_writeRegMout, -- entradas
    s_ALUOutW, s_readDataW, s_writeRegW,           -- saidas
    -- unidade de controle
    s_regWriteM, s_memToRegM, -- entradas
    s_regWriteW, s_memToRegW  -- saidas
  );

  wb: writeback port map (
    clk, s_ALUOutW, s_readDataW, s_writeRegW, s_memToRegW, -- entradas
    s_resultW, s_writeRegW_out                             -- saidas
  );

  frw: forwarding port map (
    s_rsE, s_rtE, s_writeRegM, s_writeRegW, s_regWriteM, s_regWriteW, -- entradas
    s_forwardAE, s_forwardBE -- saidas
  );

  stall: stall_unit port map (
    s_rsD, s_rtD, s_rtE, s_memToRegE, -- entradas
    s_stallF, s_stallD, s_flushE      -- saidas
  );
  -- next PC logic
  pcjump <= pcplus4(31 downto 28) & instr(25 downto 0) & "00";
  pcreg: flopr generic map(32) port map(clk, reset, pcnext, pc);
  pcadd1: adder port map(pc, X"00000004", pcplus4);
  immsh: sl2 port map(signimm, signimmsh);
  pcadd2: adder port map(pcplus4, signimmsh, pcbranch);
  pcbrmux: mux2 generic map(32) port map(pcplus4, pcbranch,
                                         pcsrc, pcnextbr);
  pcmux: mux2 generic map(32) port map(pcnextbr, pcjump, jump, pcnext);

  -- register file logic
  rf: regfile port map(clk, regwrite, instr(25 downto 21),
                       instr(20 downto 16), writereg, result, srca,
				writedata);
				
  ju: jump_unit port map(srca, writedata, zero);
  
  wrmux: mux2 generic map(5) port map(instr(20 downto 16),
                                      instr(15 downto 11),
                                      regdst, writereg);
  resmux: mux2 generic map(32) port map(aluout, readdata,
                                        memtoreg, result);
  se: signext port map(instr(15 downto 0), signimm);

  -- ALU logic
  srcbmux: mux2 generic map(32) port map(writedata, signimm, alusrc,
                                         srcb);
  mainalu: alu port map(srca, srcb, alucontrol, aluout, open);
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity regfile is -- three-port register file
  port(clk:           in  std_logic;
       we3:           in  std_logic;
       ra1, ra2, wa3: in  std_logic_vector(4 downto 0);
       wd3:           in  std_logic_vector(31 downto 0);
       rd1, rd2:      out std_logic_vector(31 downto 0));
end;

architecture behave of regfile is
  type ramtype is array (31 downto 0) of std_logic_vector(31 downto 0);
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
  port(a, b: in  std_logic_vector(31 downto 0);
       y:    out std_logic_vector(31 downto 0));
end;

architecture behave of adder is
begin
  y <= a + b;
end;


library IEEE; use IEEE.STD_LOGIC_1164.all;

entity sl2 is -- shift left by 2
  port(a: in  std_logic_vector(31 downto 0);
       y: out std_logic_vector(31 downto 0));
end;

architecture behave of sl2 is
begin
  y <= a(29 downto 0) & "00";
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity signext is -- sign extender
  port(a: in  std_logic_vector(15 downto 0);
       y: out std_logic_vector(31 downto 0));
end;

architecture behave of signext is
begin
  y <= X"ffff" & a when a(15) else X"0000" & a;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;  use IEEE.STD_LOGIC_ARITH.all;

entity flopr is -- flip-flop with synchronous reset
  generic(width: integer);
  port(clk, reset: in  std_logic;
       d:          in  std_logic_vector(width-1 downto 0);
       q:          out std_logic_vector(width-1 downto 0));
end;

architecture asynchronous of flopr is
begin
  process(clk, reset) begin
    if reset then  q <= (others => '0');
    elsif rising_edge(clk) then
      q <= d;
    end if;
  end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all;

entity mux2 is -- two-input multiplexer
  generic(width: integer);
  port(d0, d1: in  std_logic_vector(width-1 downto 0);
       s:      in  std_logic;
       y:      out std_logic_vector(width-1 downto 0));
end;

architecture behave of mux2 is
begin
  y <= d1 when s else d0;
end;

library IEEE;use IEEE.STD_LOGIC_1164.all;

entity mux3 is -- three-input multiplexer
    generic (width : INTEGER);
    port (d0, d1, d2 : in STD_LOGIC_VECTOR(width - 1 downto 0);
          s : in STD_LOGIC_VECTOR(1 downto 0);
          y : out STD_LOGIC_VECTOR(width - 1 downto 0)
    );
end;

architecture behave of mux3 is
begin
    with s select
    y <= d0 when "00", 
         d1 when "01", 
         d2 when others;
end behave;

library IEEE; use IEEE.STD_LOGIC_1164.all;
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity alu is
  port(a, b:       in  std_logic_vector(31 downto 0);
       alucontrol: in  std_logic_vector(2 downto 0);
       result:     buffer std_logic_vector(31 downto 0);
       zero:       out std_logic);
end;

architecture behave of alu is
  signal condinvb, sum: std_logic_vector(31 downto 0);
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


-- Aqui comecam as minhas modificacoes @Theo

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity fetch is
  port (
    -- entradas
    clock, reset:   in std_logic;
    PCsrcM:         in std_logic;
    PCbranchM:		  in std_logic_vector(31 downto 0);
    stallF:         in std_logic;
    -- saidas
    PCplus4F:       out std_logic_vector(31 downto 0);
    instrF:         out std_logic_vector(31 downto 0)
  );
end;

architecture struct of fetch is

  component adder
    port(a, b: in  std_logic_vector(31 downto 0);
        y:    out std_logic_vector(31 downto 0));
  end component;

	component mux2 generic(width: integer);
		port(d0, d1: in  std_logic_vector(width-1 downto 0);
				s:      in  std_logic;
				y:      out std_logic_vector(width-1 downto 0));
  end component;
  
	component floprEN generic(width: integer);
		port(clk, reset: 	in std_logic;
				enable: 		    in std_logic;
				d:          		in std_logic_vector(width-1 downto 0);
				q:          		out std_logic_vector(width-1 downto 0));
	end component;
  
	component imem
		port(a:  in  std_logic_vector(5 downto 0);
				rd: out std_logic_vector(31 downto 0));
  end component;
  
  signal s_PCsrcM, s_enable: std_logic;
  signal s_PCnext, s_PCF, s_PCbranchM, s_PCplus4F, s_instrF: std_logic_vector(31 downto 0);

begin

  s_PCsrcM <= PCsrcM;
  s_PCbranchM <= PCbranchM;
  s_enable <= not stallF;

  -- carrega instrucao
  i_mem: imem port map (s_PCF(7 downto 2), s_instrF);

  -- faz decisão de branch
  mux_branch: mux2 generic map (32) 
	port map (s_PCplus4F, s_PCbranchM, s_PCsrcM, s_PCnext);

  -- soma 4 ao Program Counter
  add_4: adder port map (s_PCF, X"00000004", s_PCplus4F);

  -- Guarda Program Counter, no registrador reg_PCF, para acesso do Stall
  reg_PCF: floprEN generic map(32)
	port map (clock, reset, s_enable, s_PCnext, s_PCF);
  
  PCplus4F <= s_PCplus4F;
  instrF <= s_instrF;

end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity regaux1 is
	port(
      -- entradas
      clock, clear:         in std_logic;
      stallD:	              in std_logic;
      PCplus4F, instrF:	  	in std_logic_vector(31 downto 0);
      -- saidas
			PCplus4D, instrD:		  out std_logic_vector(31 downto 0));
end;

architecture struct of regaux1 is 

	component floprEN generic(width: integer);
		port(
        clk, reset: 	in std_logic;
				enable: 		in std_logic;
				d:          		in std_logic_vector(width-1 downto 0);
				q:          		out std_logic_vector(width-1 downto 0));
	end component;

  signal s_enable:  std_logic;
begin

  s_enable <= not stallD;

  -- passa o PCplus4 do fetch pra o decode
  reg_PCplus4: floprEN generic map(32)
    port map (clock, clear, s_enable, PCplus4F, PCplus4D);
    
  -- passa a instr do fetch pra o decode
  reg_instr: floprEN generic map(32)
    port map (clock, clear, s_enable, instrF, instrD);

end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity decode is 
  
  port (
    -- entradas
    clock, reset:       in std_logic;
    regWriteW:          in std_logic;
    writeRegW:          in std_logic_vector(4 downto 0);
    instrD, PCplus4Din: in std_logic_vector(31 downto 0);
    resultW:            in std_logic_vector(31 downto 0);
    -- saidas
    PCplus4Dout:        out std_logic_vector(31 downto 0);
    RD1, RD2:           out std_logic_vector(31 downto 0);
    signImmD:           out std_logic_vector(31 downto 0);
    opcode, funct:      out std_logic_vector(5 downto 0);
    rsD, rtD, rdD, rdE: out std_logic_vector(4 downto 0)
  );
end;

architecture struct of decode is 

	component regfile
		port(clk:          in  STD_LOGIC;
				we3:           in  STD_LOGIC; -- write enable
				ra1, ra2, wa3: in  STD_LOGIC_VECTOR(4 downto 0); -- read port 1, 2, write port
				wd3:           in  STD_LOGIC_VECTOR(31 downto 0); -- data to write
				rd1, rd2:      out STD_LOGIC_VECTOR(31 downto 0)); -- 
  end component;
  
	component signext
		port(a: in  STD_LOGIC_VECTOR(15 downto 0);
				y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;

  signal s_PCplus4, s_RD1, s_RD2, s_signImmD: std_logic_vector(31 downto 0);
  
  begin
    -- decodifica a instrucao
    reg_file: regfile port map (clock, regWriteW, instrD(25 downto 21), instrD(20 downto 16), writeRegW, resultW, s_RD1, s_RD2);

    sign_ext: signext port map (instrD(15 downto 0), s_signImmD);
    
    s_PCplus4 <= PCplus4Din;
    PCplus4Dout <= s_PCplus4;
    
    RD1 <= s_RD1;
		RD2 <= s_RD2;
		rsD <= instrD(25 downto 21);
		rtD <= instrD(20 downto 16);
		rdE <= instrD(15 downto 11);
    opcode <= instrD(31 downto 26);
    funct <= instrD(5 downto 0);
    signImmD <= s_signImmD;
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity regaux2 is
  port (
    clock, flushE: in std_logic;
    -- fluxo de dados:
      -- entradas
      PCplus4D:       in std_logic_vector(31 downto 0);
      signImmD:       in std_logic_vector(31 downto 0); 
      rd1D, rd2D:     in std_logic_vector(31 downto 0);
      rsD, rtD, rdD:  in std_logic_vector(4 downto 0);
      -- saidas
      PCplus4E:       out std_logic_vector(31 downto 0);
      signImmE:       out std_logic_vector(31 downto 0); 
      rd1E, rd2E:     out std_logic_vector(31 downto 0);
      rsE, rtE, rdE:  out std_logic_vector(4 downto 0);
    -- unidade de controle:
      -- entradas
      regWriteD:      in std_logic;
      memToRegD: 	    in std_logic;
      memWriteD:      in std_logic; 
      branchD:        in std_logic;
      ALUControlD:		in std_logic_vector(2 downto 0);
      ALUSrcD: 		    in std_logic;
			regDstD: 				in std_logic;
      -- saidas
      regWriteE:      out std_logic;
      memToRegE: 	    out std_logic;
      memWriteE:      out std_logic; 
      branchE:        out std_logic;
      ALUControlE:		out std_logic_vector(2 downto 0);
      ALUSrcE: 		    out std_logic;
			regDstE: 				out std_logic
  );
end;

architecture struct of regaux2 is 

	component floprEN generic(width: integer);
		port(
        clk, reset: 	in std_logic;
				enable: 		in std_logic;
				d:          		in std_logic_vector(width-1 downto 0);
				q:          		out std_logic_vector(width-1 downto 0));
	end component;
  
  signal s_rdE, s_rdE_aux: std_logic_vector(63 downto 0);
  signal s_rstdE, s_rstdE_aux: std_logic_vector(14 downto 0);
  signal s_ucE, s_ucE_aux: std_logic_vector(8 downto 0);
  
  begin
  reg_rd1: floprEN generic map (32)
  port map(clock, flushE, '1', rd1D, rd1E);

  reg_rd2: floprEN generic map (32)
  port map(clock, flushE, '1', rd2D, rd2E);

  -- rs, rt e rd
  s_rstdE_aux <= rsD & rtD & rdD;
  
  reg_rstd: floprEN	generic map (15)
    port map(clock, flushE, '1', s_rstdE_aux, s_rstdE);     
  
  -- signImm 
  reg_sig: floprEN generic map (32)
    port map(clock, flushE, '1', signImmD, signImmE);
  
  -- PCplus4
  reg_PCplus4: floprEN generic map (32)
    port map(clock, flushE, '1', PCplus4D, PCplus4E);
  
  -- sinais da UC   
  s_ucE_aux <= regWriteD & memToRegD & memWriteD & branchD & ALUControlD & ALUSrcD & regDstD;
  
  regUC: floprEN generic map (9) 
	port map(clock, flushE, '1', s_ucE_aux, s_ucE);

  -- separação de sinais que foram colapsados
	rsE <= s_rstdE(14 downto 10);
	rtE <= s_rstdE(9 downto 5);
  rdE <= s_rstdE(4 downto 0);
  regWriteE <= s_ucE(8);
  memToRegE <= s_ucE(7);	
  memWriteE <= s_ucE(6);
  branchE <= s_ucE(5);   
  ALUControlE <= ALUControlD;
  ALUSrcE	<= s_ucE(1);
  regDstE <= s_ucE(0);			
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity execute is
  port (
    clock:            in std_logic;
    -- entradas
      -- fluxo de dados:
      rtE, rdE:             in std_logic_vector(4 downto 0);
      rd1E, rd2E:           in std_logic_vector(31 downto 0);
      signImmE:             in std_logic_vector(31 downto 0);
      PCplus4E:             in std_logic_vector(31 downto 0);
      resultW:              in std_logic_vector(31 downto 0);
      ALUOutM:              in std_logic_vector(31 downto 0);
      forwardAE, forwardBE: in std_logic_vector(1 downto 0);
      -- unidade de controle:
      regDstE:              in std_logic;
      ALUSrcE:              in std_logic;
      ALUControlE:          in std_logic_vector(2 downto 0);
    -- saidas
      zeroE:                out std_logic;
      ALUOutE:              out std_logic_vector(31 downto 0);
      writeDataE:           out std_logic_vector(31 downto 0);
      writeRegE:            out std_logic_vector(4 downto 0);
      PCbranchE:            out std_logic_vector(31 downto 0)
  );
end;

architecture struct of execute is
  
  component alu
		port(a, b:       in  STD_LOGIC_VECTOR(31 downto 0);
				alucontrol: in  STD_LOGIC_VECTOR(2 downto 0);
				result:     buffer STD_LOGIC_VECTOR(31 downto 0);
				zero:       out STD_LOGIC);
  end component;
  
	component mux2 generic(width: integer);
		port(d0, d1: in  STD_LOGIC_VECTOR(width-1 downto 0);
				s:      in  STD_LOGIC;
				y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;

  component adder
    port(a, b: in  std_logic_vector(31 downto 0);
        y:    out std_logic_vector(31 downto 0));
  end component;

	component sl2
		port(a: in  STD_LOGIC_VECTOR(31 downto 0);
				y: out STD_LOGIC_VECTOR(31 downto 0));
  end component;

  component mux3 is generic(width: integer);
    port(d0, d1, d2: in  STD_LOGIC_VECTOR(width-1 downto 0);
       s:      in  STD_LOGIC_VECTOR(1 downto 0);
       y:      out STD_LOGIC_VECTOR(width-1 downto 0));
  end component;
  
  signal s_writeDataE, s_srcAE, s_srcBE, s_SL2out: std_logic_vector(31 downto 0);

  begin
    alu_alu: alu port map (s_srcAE, s_srcBE, ALUControlE, ALUOutE, zeroE);

    muxALU: mux2 generic map (32)
      port map (s_writeDataE, signImmE, ALUSrcE, s_srcBE);

    muxReg: mux2 generic map (5)
      port map (rtE, rdE, regDstE, writeRegE);

    shiftl2: sl2 port map (signImmE, s_SL2out);

    add: adder port map (s_SL2out, PCplus4E, PCbranchE);
    
    mux_forwA: mux3 generic map (32) 
      port map (rd1E, resultW, ALUOutM, forwardAE, s_srcAE);

    mux_forwB: mux3 generic map (32) 
      port map (rd2E, resultW, ALUOutM, forwardBE, s_writeDataE);

end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity regaux3 is
  port (
  clock, clear:            in std_logic;
  -- fluxo de dados
    -- entradas
    zeroE:          in std_logic;
    ALUoutE:        in std_logic_vector(31 downto 0);
    writeDataE:     in std_logic_vector(31 downto 0);
    writeRegE:      in std_logic_vector(4 downto 0);
    PCbranchE:      in std_logic_vector(31 downto 0);
    -- saidas
    zeroM:          out std_logic;
    ALUoutM:        out std_logic_vector(31 downto 0);
    writeDataM:     out std_logic_vector(31 downto 0);
    writeRegM:      out std_logic_vector(4 downto 0);
    PCbranchM:      out std_logic_vector(31 downto 0);
  -- unidade de controle
    -- entradas
    regWriteE:      in std_logic;
    memToRegE: 	    in std_logic;
    memWriteE:      in std_logic; 
    branchE:        in std_logic;
    -- saidas
    regWriteM:      out std_logic;
    memToRegM: 	    out std_logic;
    memWriteM:      out std_logic; 
    branchM:        out std_logic
  );

end;

architecture struct of regaux3 is

	component floprEN generic(width: integer);
		port(clk, reset: 	in STD_LOGIC;
				enable: 	in STD_LOGIC;
				d:          in STD_LOGIC_VECTOR(width-1 downto 0);
				q:          out STD_LOGIC_VECTOR(width-1 downto 0));
	end component;  
  
  signal s_FD, s_FD_aux: std_logic_vector(5 downto 0);
  signal s_UC, s_UC_aux: std_logic_vector(3 downto 0);

  begin
  s_FD_aux <= zeroE & writeRegE;

  reg_zero_wReg: floprEN generic map(6)
    port map(clock, clear, '1', s_FD_aux, s_FD);
  
  reg_ALUout: floprEN generic map(32)
    port map(clock, clear, '1', ALUOutE, ALUoutM);

  reg_writeData: floprEN generic map(32)
    port map(clock, clear, '1', writeDataE, writeDataM);

  reg_PCbranch: floprEN generic map(32)
    port map(clock, clear, '1', PCbranchE, PCbranchM);

	s_UC_aux <= regWriteE & memToRegE & memWriteE & branchE;
    
	reg_UC: floprEN generic map(4)
      port map(clock, clear, '1', s_UC_aux, s_UC);
	  
	zeroM <= s_FD(5);
    writeRegM <= s_FD(4 downto 0);
	
    regWriteM <= s_UC(3);
    memToRegM <= s_UC(2);
    memWriteM <= s_UC(1);
    branchM <= s_UC(0); 
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity memory is
  port (
    clock:            in std_logic;
    -- entradas
      -- fluxo de dados:
      ALUOutM:        in std_logic_vector(31 downto 0);
      writeDataM:     in std_logic_vector(31 downto 0);
      writeRegM:      in std_logic_vector(4 downto 0);
      PCbranchM:      in std_logic_vector(31 downto 0);
      -- unidade de controle:
      memWriteM:      in std_logic;
    -- saidas
      ALUOutW:        out std_logic_vector(31 downto 0);
      rdM:            out std_logic_vector(31 downto 0);
      writeRegW:      out std_logic_vector(4 downto 0); 
      PCbranchF:      out std_logic_vector(31 downto 0)
  );
end;

architecture struct of memory is

	component dmem
		port(clk, we:  in STD_LOGIC;
				a, wd:    in STD_LOGIC_VECTOR(31 downto 0);
				rd:       out STD_LOGIC_VECTOR(31 downto 0));
	end component;
  
  signal s_PCbranch, s_ALUout: std_logic_vector(31 downto 0);
  signal s_writeReg: std_logic_vector(4 downto 0);

  begin
    data_memory: dmem port map(clock, memWriteM, ALUOutM, writeDataM, rdM);

    s_PCbranch <= PCbranchM;
    PCbranchF <= s_PCbranch;

    s_ALUout <= ALUOutM;
    ALUOutW <= s_ALUout;

    s_writeReg <= writeRegM;
    writeRegW <= s_writeReg;
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity regaux4 is
  port (
  clock, clear:            in std_logic;
  -- fluxo de dados
    -- entradas
    ALUOutM:        in std_logic_vector(31 downto 0);
    rdM:            in std_logic_vector(31 downto 0);
    writeRegM:      in std_logic_vector(4 downto 0); 
    -- saidas
    ALUOutW:        out std_logic_vector(31 downto 0);
    readDataW:      out std_logic_vector(31 downto 0);
    writeRegW:      out std_logic_vector(4 downto 0); 
  -- unidade de controle
    -- entradas
    regWriteM:      in std_logic;
    memToRegM: 	    in std_logic;
    -- saidas
    regWriteW:      out std_logic;
    memToRegW: 	    out std_logic
  );

end;

architecture struct of regaux4 is

	component floprEN generic(width: integer);
		port(clk, reset: 	in STD_LOGIC;
				enable: 		in STD_LOGIC;
				d:          		in STD_LOGIC_VECTOR(width-1 downto 0);
				q:          		out STD_LOGIC_VECTOR(width-1 downto 0));
	end component;  
  
  signal s_UC, s_UC_aux: std_logic_vector(1 downto 0);

  begin
    reg_FD1: floprEN 	generic map(32)
		port map(clock, clear, '1', ALUOutM, ALUoutW);
		
    reg_FD2: floprEN 	generic map(32)
		port map(clock, clear, '1', rdM, readDataW);
		
    reg_FD3: floprEN 	generic map(5)
		port map(clock, clear, '1', writeRegM, writeRegW);

	s_UC_aux <= regWriteM & memToRegM;
	
    reg_UC: floprEN 	generic map(2)
		port map(clock, clear, '1', s_UC_aux, s_UC);
    
    regWriteW <= s_UC(1);
    memToRegW <= s_UC(0);
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 

entity writeback is
  port (
    clock:            in std_logic;
    -- entradas
      -- fluxo de dados:
      ALUOutW:        in std_logic_vector(31 downto 0);
      readDataW:      in std_logic_vector(31 downto 0);
      writeRegW_in:   in std_logic_vector(4 downto 0);       
      -- unidade de controle:
      memToRegW: 	    in std_logic;
    -- saidas
      resultW:        out std_logic_vector(31 downto 0);
      writeRegW_out:  out std_logic_vector(4 downto 0)
  );
end;

architecture struct of writeback is
  
  component mux2 generic(width: integer);
		port(d0, d1: in  std_logic_vector(width-1 downto 0);
				s:      in  std_logic;
				y:      out std_logic_vector(width-1 downto 0));
  end component;

  begin
    mux_result: mux2 generic map(32) port map (ALUOutW, readDataW, memToRegW, resultW);
	  writeRegW_out <= writeRegW_in;
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity forwarding is
  port (
    rsE, rtE:               in std_logic_vector (4 downto 0);
    writeRegM, writeRegW:   in std_logic_vector (4 downto 0);
    regWriteM, regWriteW:   in std_logic;
    forwardAE, forwardBE:   out std_logic_vector (1 downto 0)
  );
end;

architecture behave of forwarding is

  begin
    process(all) begin
      if rsE /= "0000" and rsE = writeregM and regWriteM = '1' then
        forwardAE <= "10";
      elsif rsE /= "0000" and rsE = writeregW and regWriteW = '1' then
        forwardAE <= "01";
      else 
        forwardAE <= "00";
      end if;
      if rtE /= "0000" and rtE = writeregM and regWriteM = '1' then
        forwardBE <= "10";
      elsif rtE /= "0000" and rtE = writeregW and regWriteW = '1' then
        forwardBE <= "01";
      else 
        forwardBE <= "00";
      end if;   
	  end process;
  end;

library IEEE; use IEEE.STD_LOGIC_1164.all; 
use IEEE.NUMERIC_STD_UNSIGNED.all;

entity stall_unit is
  port (
    rsD, rtD, rtE:            in std_logic_vector (4 downto 0);
    memToRegE:                in std_logic;
    stallF, stallD, flushE:   out std_logic
  );
end;

architecture behave of stall_unit is

  signal lwstall: std_logic;
  begin
    process(all) begin
      if (rsD = rtE or rtD = rtE) and memToRegE = '1' then
        lwstall <= '1';
      else
        lwstall <= '0';
      end if;

      stallF <= lwstall;
      stallD <= lwstall;
      flushE <= lwstall;
	
	end process;
end;

library IEEE; use IEEE.STD_LOGIC_1164.all; use IEEE.STD_LOGIC_ARITH.all;

entity jump_unit is
  port(
    register1:  in  STD_LOGIC_VECTOR(31 downto 0); 
    register2:  in  STD_LOGIC_VECTOR(31 downto 0);
    equal: out STD_LOGIC);
end entity jump_unit;

architecture behavioral of jump_unit is

begin
  equal <= '1' when (register1 = register2) 
        else '0';
end architecture behavioral;
