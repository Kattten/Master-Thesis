library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
use IEEE.math_real.all;

use ieee.fixed_float_types.all; 
use ieee.fixed_pkg.all;

entity pid_fixed is
    Port ( ADC_DATA_0 : in  STD_LOGIC_VECTOR (11 downto 0); 
	        ADC_DATA_2 : in  STD_LOGIC_VECTOR (11 downto 0); 
           Output_DATA : out  STD_LOGIC_VECTOR (11 downto 0); 
			  Att_Out: out  STD_LOGIC_VECTOR (5 downto 0); 
			  DAC_DATA_0 : out  STD_LOGIC_VECTOR (11 downto 0); 
			  DAC_DATA_2 : out  STD_LOGIC_VECTOR (11 downto 0); 
			  DAC_ERROR:    out  STD_LOGIC_VECTOR (11 downto 0); 
			  DAC_i: out  STD_LOGIC_VECTOR (11 downto 0); 
			  DAC_p: out  STD_LOGIC_VECTOR (11 downto 0); 
           CLK1 : in STD_LOGIC);
end pid_fixed;
architecture Behavioral of pid_fixed is
    type statetypes is (Reset,		--user defined type to determine the flow of the system
			CalculateNewError,
			CalculatePID,
			DivideKg,
			Write2DAC,                              
			SOverload,
			ConvDac);	                             
    
    signal state,next_state : statetypes := Reset;     
    signal Kp : real := 1.0;		--proportional constant
    signal Kd : real :=0.0;		--differential constant
     Signal Ki : real :=0.5;		--integral constant
    signal SetVal : real := 1860.0; -- assignment of input sarting value voltage in mV
	 signal Output: sfixed(11 downto -11) :=to_sfixed(SetVal, 11,-11);
	 signal Output_new: sfixed(11 downto 0) :=to_sfixed(SetVal, 11,0);
	    signal inter: sfixed(11 downto -11) := (others=>'0');		--intermediate signal
        signal sAdc_0 : Integer;	--stores the integer converted value of the ADC input Channel 0
	  signal sAdc_2 : Integer;	--stores the integer converted value of the ADC input Channel 2
    signal Error: sfixed(11 downto -11) := (others=>'0');		--Stores the deviation of the input from the set point
    signal p,i,d : sfixed(11 downto -11) := (others=>'0');	--Contain the proportional, derivative and integral errors respectively
	 signal sADC_0_sf: sfixed(11 downto -11); -- to store ADC Channel 0 value in sfixed format
	 signal sADC_2_sf: sfixed(11 downto -11); -- to store ADC Channel 2 value in sfixed format
	  signal sADC_2_cor: sfixed(11 downto -11);-- to store ADC Channel 0 value in sfixed format corrected for 25 mV diffference in detectors RF1 and RF2
    signal DacDataCarrier : std_logic_vector (11 downto 0); --contains the binary converted value to be output to the DAC
	 signal SetVal_sf : sfixed(11 downto -11); --to store the setvalue/starting value in sfixed format
	 signal sADC_sf: sfixed(11 downto -11);
	   signal Att_word: std_logic_vector (5 downto 0) := "000000"; -- the code word to send to Attenuator
		signal sADC_0_read: std_logic_vector (11 downto 0); --standard logic vector for control of conversion to sfixed of the ADC CH0 correct reading
	 signal sADC_2_read: std_logic_vector (11 downto 0); --standard logic vector for control of conversion to sfixed of the ADC CH2 correct reading
	 signal ERROR_to_DAC: std_logic_vector (11 downto 0); --standard logic vector for sending ERROR to the ouput port
	  signal Toler: real :=10.0; -- difference in detector reading corresponding to 0.5dB
	 signal Toler_sf: sfixed (11 downto -11); -- for storing in sfixed format of the difference in detector reading corresponding to 0.5dB
	 signal Output_slv: std_logic_vector (11 downto 0); --for sending different inermediate voltage values (in mV) to the output port
	 signal i_slv: std_logic_vector (11 downto 0); --for sending integration correction  voltage values (in mV) to the output port
	 signal p_slv: std_logic_vector (11 downto 0);--for sending proprtional  correction  voltage values (in mV) to the output port
	  signal Output_Old : sfixed(11 downto -11) :=to_sfixed(SetVal, 11,-11); --intermediate value for the output
      signal Error_Old : sfixed(11 downto -11) := (others=>'0'); --intermediate value for the Error
     
begin
PROCESS(CLK1,state)		--sensitive to Clock and current state
     
		
     BEGIN
		   
         IF CLK1'EVENT AND CLK1='1' THEN  
				state <= next_state;
         END IF;
			
         case state is
		 when Reset =>
			next_state <= CalculateNewError;
			sAdc_0 <= to_integer(unsigned(ADC_DATA_0));  --Get the input for PID
			sAdc_2 <= to_integer(unsigned(ADC_DATA_2));  --Get the input for PID
			sADC_0_sf<=to_sfixed(sADC_0,sADC_0_sf);
			sADC_2_sf<=to_sfixed(sADC_2,sADC_2_sf);
			Toler_sf<=to_sfixed(Toler,Toler_sf);
			SetVal_sf<=to_sfixed(SetVal,SetVal_sf);
			sADC_2_cor<=resize(sADC_2_sf-25.0,sADC_2_sf);
			
			Error_Old <= Error;  --Capture old error
			Output_Old <= Output;    --Capture old PID output
			
		  when CalculateNewError =>  
			next_state <= CalculatePID;
			inter <= resize(sADC_0_sf-sADC_2_cor, inter); --Calculate Error
			Error<=inter;
		  
		  when CalculatePID =>
			next_state <= DivideKg;
			p <= resize(Kp*Error,p);              --Calculate PID 
			i <= resize(Ki*(Error+Error_Old),i);
			--d <= resize(Kd *(Error-Error_Old),8,-1);  
   
   	if resize(abs((p+i)),Toler_sf)<Toler_sf  
	   then 
	   Att_word <= "000000"; 
		end if;	
		  
      if resize(abs((p+i)),Toler_sf)>=resize(Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(2*Toler_sf,Toler_sf) 
		then 
		Att_word <= "000010";-- 2 steps
	 	end if;
		
		if resize(abs((p+i)),Toler_sf)>=resize(2*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(3*Toler_sf,Toler_sf) 
		then 
		Att_word <= "000011";--3 steps
	  	end if; 
			  
	   if resize(abs((p+i)),Toler_sf)>=resize(3*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(4*Toler_sf,Toler_sf) 
		then
		Att_word <= "000100";-- 4 steps
		end if; 
		
		if resize(abs((p+i)),p)>=resize(4*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(5*Toler_sf,Toler_sf) 
		then 
		Att_word <= "000100";-- 4 steps
		end if; 
		
		if resize(abs((p+i)),Toler_sf)>=resize(5*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(6*Toler_sf,Toler_sf) 
		then 
		Att_word <= "000101";-- 5 steps
	  	end if;  
		
		if resize(abs((p+i)),Toler_sf)>=resize(6*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(7*Toler_sf,Toler_sf)
		then 
		Att_word <= "000111";-- 7 steps
		end if;
					 
      if resize(abs((p+i)),Toler_sf)>=resize(7*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(8*Toler_sf,Toler_sf) 
		then
		Att_word <= "001000";-- 8 steps
		end if;  		
		
	   if resize(abs((p+i)),p)>=resize(8*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(9*Toler_sf,Toler_sf) 
		then 
		Att_word <= "001001"; -- 9 steps
	  	end if; 
		
		if resize(abs((p+i)),Toler_sf)>=resize(9*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(10*Toler_sf,Toler_sf) 
		then 
		Att_word <= "001010";-- 10 steps
		end if;  
		
				if resize(abs((p+i)),Toler_sf)>=resize(11*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(12*Toler_sf,Toler_sf) 
	   then 
		Att_word <= "001011";--11 steps
		end if;
		
		if resize(abs((p+i)),Toler_sf)>=resize(12*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(13*Toler_sf,Toler_sf) 
		then 
		Att_word <= "001100";-- 12 steps
		end if;
		
	   if resize(abs((p+i)),Toler_sf)>=resize(13*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(14*Toler_sf,Toler_sf) 
		then 
	   Att_word <= "001101";-- 13 steps
	  	end if;
		
		if resize(abs((p+i)),Toler_sf)>=resize(14*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(15*Toler_sf,Toler_sf) 
		then 
		Att_word <= "001110";--14 steps
		end if;
				
		if resize(abs((p+i)),Toler_sf)>=resize(15*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(16*Toler_sf,Toler_sf)
		then 
		Att_word <= "010000";-- 16 steps
	   end if;
	  
	  	if resize(abs((p+i)),Toler_sf)>=resize(16*Toler_sf,Toler_sf)  and resize(abs((p+i)),Toler_sf)<resize(17*Toler_sf,Toler_sf)
		then  
		Att_word <= "010001";--17 steps
	   end if;
	  
		if resize(abs((p+i)),p)>=resize(17*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(18*Toler_sf,Toler_sf) 
		then 
		Att_word <= "010010";-- 18 steps
	  	end if;  
			
		if resize(abs((p+i)),p)>=resize(18*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(20*Toler_sf,Toler_sf)
		then 
		Att_word <= "010011";-- 19 steps
		end if;
		
		if resize(abs((p+i)),p)>=resize(20*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(21*Toler_sf,Toler_sf)
		then 
		Att_word <= "010101";-- 21 steps
     	end if;
		
		if resize(abs((p+i)),p)>=resize(21*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(22*Toler_sf,Toler_sf) 
		then 
		Att_word <= "010110";-- 22 step
		end if;
		
		if resize(abs((p+i)),p)>=resize(22*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(23*Toler_sf,Toler_sf) 
		then 
		Att_word <= "010111"; ---23 steps
	   end if;
	  
	   if resize(abs((p+i)),p)>=resize(23*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(24*Toler_sf,Toler_sf) 
		then 
		Att_word <= "011000";-- 24 steps
      end if;
		
		 if resize(abs((p+i)),p)>=resize(24*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(25*Toler_sf,Toler_sf)
		then 
		Att_word <= "011001";-- 25 steps
		end if;
		
		 if resize(abs((p+i)),p)>=resize(25*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(26*Toler_sf,Toler_sf) 
		then 
		Att_word <= "011010";-- 26 steps
     	end if;
		
	   if resize(abs((p+i)),p)>=resize(26*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(27*Toler_sf,Toler_sf) 
		then 
		Att_word <= "011011";-- 27 steps
	 	end if;
	  
	   
	   if resize(abs((p+i)),p)>=resize(27*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(28*Toler_sf,Toler_sf)
		then 
		Att_word <= "011100";-- 28 steps
     	end if;			
				
	   if resize(abs((p+i)),p)>=resize(28*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(29*Toler_sf,Toler_sf) 
  		then 
		Att_word <= "011101";-- 29 steps
     	end if;	
	
 	   if resize(abs((p+i)),p)>=resize(29*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(30*Toler_sf,Toler_sf)
		then 
		Att_word <= "011110";-- 30 steps
     	end if;		
		
	   if resize(abs((p+i)),p)>=resize(30*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(31*Toler_sf,Toler_sf) 
		then 
		Att_word <= "011111";-- 31 steps
     	end if;	
		
	   if resize(abs((p+i)),p)>=resize(31*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(32*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100000";-- 32 steps
     	end if;	
		
	   if resize(abs((p+i)),p)>=resize(32*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(33*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100001";-- 33 steps
     	end if;	
				
   	if resize(abs((p+i)),p)>resize(33*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(34*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100010";--34 steps
     	end if;  
	  			
  	   if resize(abs((p+i)),p)>resize(34*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(35*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100011";-- 35 steps
     	end if; 
		 
	   if resize(abs((p+i)),p)>resize(35*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(36*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100100";--36 steps
     	end if; 
		
	   if resize(abs((p+i)),p)>resize(36*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(37*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100101";--37 steps
		end if; 
		
	   if resize(abs((p+i)),p)>resize(37*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(38*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100110";--38 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(38*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(39*Toler_sf,Toler_sf) 
		then 
		Att_word <= "100111";--39 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(39*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(40*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101000";--40 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(40*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(41*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101001";-- 41 steps
     	end if; 
				
	   if resize(abs((p+i)),p)>resize(41*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(42*Toler_sf,Toler_sf)
		then 
		Att_word <= "101010";--42 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(42*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(43*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101011";--43 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(43*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(44*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101100";--44 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(44*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(45*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101101";--45 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(45*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(46*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101110";--46 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(46*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(47*Toler_sf,Toler_sf) 
		then 
		Att_word <= "101111";--47 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(47*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(48*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110000";--48 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(48*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(49*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110001";--49 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(49*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(50*Toler_sf,Toler_sf)
		then 
		Att_word <= "110010";--50 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(50*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(51*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110011";--51 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(51*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(52*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110100";--52 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(52*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(53*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110101";--53 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(53*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(54*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110110";--54 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(54*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(55*Toler_sf,Toler_sf) 
		then 
		Att_word <= "110111";--55 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(55*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(56*Toler_sf,Toler_sf)
		then 
		Att_word <= "111000";--56 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(56*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(57*Toler_sf,Toler_sf) 
		then 
		Att_word <= "111001";--57 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(57*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(58*Toler_sf,Toler_sf) 
		then 
		Att_word <= "111010";--58 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(58*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(59*Toler_sf,Toler_sf) 
		then 
		Att_word <= "111011";--59 steps
     	end if;
		
	   if resize(abs((p+i)),p)>resize(59*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(60*Toler_sf,Toler_sf) 
		then 
		Att_word <= "111100";--60 steps
     	end if;
	 	
	   if resize(abs((p+i)),p)>resize(60*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(61*Toler_sf,Toler_sf)
		then 
		Att_word <= "111101";--61 steps
     	end if;
		
	if resize(abs((p+i)),p)>resize(61*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(62*Toler_sf,Toler_sf) 
		then 
		Att_word <= "111110";--62 steps
     	end if;
		
	if resize(abs((p+i)),p)>resize(62*Toler_sf,Toler_sf)  and resize(abs((p+i)),p)<resize(63*Toler_sf,Toler_sf)
		then 
		Att_word <= "111111";--63 steps
     	end if;
		
		  when DivideKg =>
			next_state <= SOverload;
			
			sADC_0_read <= to_slv(resize(sADC_0_sf,11,0));
		   sADC_2_read <= to_slv(resize(sADC_2_sf,11,0));
		   ERROR_to_DAC <= to_slv(resize(abs(Error_Old),11,0));
		  
		  when SOverload =>
			next_state <=ConvDac;	
					
		  when ConvDac =>        		
			next_state <=Write2DAC;
			i_slv<= to_slv(resize(abs(i),11,0));
			p_slv<= to_slv(resize(abs(p),11,0));
			Output_slv <= to_slv(resize(abs((p+i)),11,0)); -- p+i is convertedto SLV
			Output_new<=to_sfixed(Output_slv,Output_new);  -- p+i in SLV format is converted back to fixed format
			DacDataCarrier <= to_slv(Output_new); -- again p+i is converted from fixed to SLV
			
	
			
		  when Write2DAC =>				--send output to the DAC
			next_state <= Reset;
			DAC_DATA_0 <= sADC_0_read;
	      DAC_DATA_2 <= sADC_2_read;
	      DAC_ERROR <=ERROR_to_DAC;
			Output_DATA <=DacDataCarrier; --p+i is written to outpu port Output_DATA
			DAC_i<=i_slv;
			DAC_p<=p_slv;	
			Att_Out <=Att_word;
			
			end case;
	                        
END PROCESS;	--end of process
end Behavioral;		--end of Architecture
