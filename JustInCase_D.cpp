#include<iostream>
#include<string>
#include<vector>
#include<bitset>
#include<fstream>

using namespace std;

#define ADDU 1
#define SUBU 3
#define AND 4
#define OR  5
#define NOR 7
#define MemSize 20000

/*
 * The Main Function Contains 4 Classes, instructionMemory, Memory, Registry, and ALU
 *
 * instructionMemory: Used to store instructions from text file
 * Registry: Used to simulate Registry of MIPS Computer
 * Memory: Used to simulate Memory of MIPS Computer
 * ALU: Used to simulate ALU, do R-Type Operations

 */

/* instructionMemory Class
 * The instructionMemory constructor  reads a series of instructions from Instructions.txt,
 * and stores them in a bitset<32> Vector.
 *
 * The ReadMemory Function allows this vector to be read by other classes
 * */
class instructionMemory
{
private:
    //Contains vector of instructions from instruction file
    vector<bitset<8>> instructionMem;
public:
    bitset<32> Instruction;
    //Initializes instruction memory vector
    instructionMemory() {
        instructionMem.resize(MemSize);
        ifstream imem;
        string line;
        int i=0;
        imem.open("Instructions.txt");
        if (imem.is_open())
        {
            while (getline(imem,line))
            {
                instructionMem[i] = bitset<8>(line);
                i++;
            }
        }
        else cout<<"Unable to open file";
        imem.close();
    }

    //Reads instruction from instruction memory
    bitset<32> ReadMemory (bitset<32> ReadAddress)
    {
        unsigned long count = ReadAddress.to_ulong();
        int index = (int)count;
        string inst;
        for(int i=0;i<4;i++){

            inst += instructionMem[index].to_string();
            index++;
        }
        bitset<32> Inst(inst);
        Instruction = Inst;

        return Instruction;
    }
};

/* Registry Class
 * The constructor creates an empty bitset<32> vector to simulate an empty registry
 *
 * The ReadWrite function either reads from data varialbles, or writes to them, depending on
 * the inputs
 *
 * The OutputRegistry function writes from the Registry vector to a text file
 *
 */
class Registry
{
private:
    vector<bitset<32>> Registers;
public:
    bitset<32> Read1, Read2;
    //bitset<32> ReadData1, ReadData2;
    //Creates 32 empty registers
    Registry()
    {
        Registers.resize(32);
        Registers[0] = bitset<32> (0);
    }
//  void ReadWrite(bitset<5> RdReg1, bitset<5> RdReg2, bitset<5> WrtReg, bitset<32> WrtData, bitset<1> WrtEnable)
    //Reads and writes from and to registers, based on WrtEnable
    void ReadWrite(bitset<5> ReadReg1, bitset<5> ReadReg2, bitset<5> WriteReg, bitset<32> WrtData, bitset<1> WrtEnable)
    {
        // implement the function by you.

        Read1 = Registers[ReadReg1.to_ulong()];   // perform read operation
        Read2 = Registers[ReadReg2.to_ulong()];

        if(WrtEnable.to_ulong() == 1){

            Registers[WriteReg.to_ulong()] = WrtData;    // perform write operation
        }
    }

    //Writes out Registry Vector to file
    void outputRegistry()
    {
        ofstream registryOut;
        //Empty out what was there before.
        registryOut.open("Registers.txt", std::ofstream::out | std::ofstream::trunc);
        registryOut.close();
        //Loop through Registry Vector
        registryOut.open("Registers.txt",std::ios_base::app);
        if (registryOut.is_open())
        {
            registryOut<<"A state of Registry:"<<endl;
            for (int j = 0; j<32; j++)
            {
                registryOut << Registers[j]<<endl;
            }
        }
        else cout<<"Unable to open file";
        registryOut.close();
    }
};

/* ALU Class:
 * The ALUOperation Function conducts the R-Type instructions.
 */

class ALU
{
public:
    bitset<32> ALUresult;
    //Does the actual operation requested by R-instructions
    bitset<32> ALUOperation (bitset<3> ALUOP, bitset<32> oprand1, bitset<32> oprand2)
    {    // implement the ALU operations by you.
        unsigned long result;

        if(ALUOP.to_string() == "001"){
            result = oprand1.to_ulong() + oprand2.to_ulong(); // addu operation
        }
        else if(ALUOP.to_string() == "011"){
            result = oprand1.to_ulong() - oprand2.to_ulong(); // subu operation
        }
        else if(ALUOP.to_string() == "100"){
            result = oprand1.to_ulong() & oprand2.to_ulong(); // and operation
        }
        else if(ALUOP.to_string() == "101"){
            result = oprand1.to_ulong() | oprand2.to_ulong(); // or operation
        }
        else if(ALUOP.to_string() == "111"){
            result = ~(oprand1.to_ulong() | oprand2.to_ulong()); // nor operation
        }

        bitset<32> res((int)result);
        ALUresult = res;
        return ALUresult;
    }
};

/* Memory Class
 * The constructor reads from the IMemory.txt file to a bitset<32> vector
 * The MemoryAccess function either returns bitset<32> values or writes to them, depending on the
 * readmem and writemem check variables
 *
 * The OutputDataMem function reads through the bitset<32> vector and writes it to Memory.txt
 * */
class Memory
{
private:
    vector<bitset<32>> Mem;
public:
    bitset<32> readdata;
    //Creates Vector to store memory values
    Memory()
    {
        Mem.resize(MemSize);
        ifstream dmem;
        string line;
        int i=0;
        dmem.open("IMemory.txt");
        if (dmem.is_open())
        {
            while (getline(dmem,line))
            {
                Mem[i] = bitset<32>(line);
                i++;
            }
        }
        else cout<<"Unable to open file";
        dmem.close();
    }

    //Reads and writes from and to memory depending on readmem/writemem binary variables
    bitset<32> MemoryAccess (bitset<32> Address, bitset<32> WriteData, bitset<1> readmem, bitset<1> writemem)
    {
        unsigned long count = Address.to_ulong();
        int index = (int) count;
        //cout << "SW Index: " << index << endl;
        if((readmem.to_ulong() == 1) && (writemem.to_ulong() == 0)) {
            string data;
            data=Mem[index].to_string();

            bitset<32> Data(data);
            readdata = Data;
        }

        else if((readmem.to_ulong() == 0) && (writemem.to_ulong() == 1)){
            // perform write operation
            bitset<32> finalResult(WriteData.to_string());
            Mem[index] = finalResult;
        }
        return readdata;
    }

    //This utility function sends results of memory vector to output text file
    void OutputDataMem()
    {
        ofstream dmemout;
        dmemout.open("Memory.txt");
        if (dmemout.is_open())
        {
            for (int j = 0; j< 32; j++)
            {
                //cout << DMem[j] << endl;
                dmemout << Mem[j]<<endl;
            }
        }
        else cout<<"Unable to open file";
        dmemout.close();
    }
};

//Utility function so we can deal with words instead of bytecode.
string TypeofInstruction(string opcode){
    if(opcode == "000000") return "R";
    else if(opcode == "000010") return "J";
    else if(opcode == "111111") return "halt";
    else if( opcode == "001001") return "addiu";
    else if(opcode == "000100") return "beq";
    else if(opcode == "100011") return "lw";
    else if(opcode == "101011") return "sw";
    else return "Invalid Opcode";
}

//Utility function so we can use words instead of bytecode.
string TypeofFunction(bitset<32> instruction){
    string aluOP = instruction.to_string().substr(29,3);
    if(aluOP == "001") return "addu";
    else if (aluOP == "011") return "subu";
    else if (aluOP == "100") return "and";
    else if (aluOP == "101") return "or";
    else if (aluOP == "111") return "nor";
    else return "Invalid AluOP";
}

//Utility function to divide instruction based on R-Type Organization Pattern (RS, RT, RD)
vector<bitset<5>> divInstruction_R(bitset<32> instruction){
    vector<bitset<5>> result(3);
    string rs = instruction.to_string().substr(6,5);
    string rt = instruction.to_string().substr(11,5);
    string rd = instruction.to_string().substr(16,5);
    bitset<5> Rd(rd);
    bitset<5> Rs(rs);
    bitset<5> Rt(rt);

    result[0] = Rs;
    result[1] = Rt;
    result[2] = Rd;

    return result;
}

//Utility function to divide instruction based on I-Type Organization Pattern (RS, RD, IMM)
vector<string> divInstruction_I(bitset<32> instruction){
    string instr = instruction.to_string();

    string rs = instruction.to_string().substr(6,5);
    string rd = instruction.to_string().substr(11,5);
    string imm = instruction.to_string().substr(16,16);

    vector<string> result(3);
    result[0] = rs;
    result[1] = rd;
    result[2] = imm;

    return result;
}

//This utility function calculates the next instruction to be read.
// Usually, this will be the next one (PC+4), but can be specified by
//Jump or BEQ instruction.
bitset<32> calc_PC(bitset<32> &programCounter)
{
    programCounter = bitset<32> (programCounter.to_ulong()+4);
    return programCounter;
}

/*Main Function*/
int main()
{
    Registry myRegistry;
    ALU myALU;
    instructionMemory myInstructions;
    Memory myMemory;

    bitset<32> programCounter = bitset<32> (0),instruction;

    string Type_instruction, Type_function;
    bitset<5> Addr_Rs, Addr_Rt, Addr_Rd;
    bitset<32> $Rs,$Rd,$Rt,AluResult;
    bitset<3> add_op(ADDU), subu_op(SUBU), and_op(AND), or_op(OR), nor_op(NOR);
    vector<bitset<5>> RAddresses;
    vector<string> IAddressess(3);
    unsigned long counter=0;

    //Loop through instructions from Instructions File until a HALT instruction is seen
    while (1)
    {
        // Fetch

        instruction = myInstructions.ReadMemory(programCounter);
        if(instruction.to_string() == "11111111111111111111111111111111") break; //halt condition


        //execute instruction

        string opcode = instruction.to_string().substr(0,6);  //cout<<"Opcode : "<<opcode<<" : ";

        string Type_instruction = TypeofInstruction(opcode);  //identifying instruction type
        cout << Type_instruction << endl;
        if(Type_instruction == "R"){
            //execute R instruction
            RAddresses = divInstruction_R(instruction);
            Addr_Rs = RAddresses[0];
            Addr_Rt = RAddresses[1];
            Addr_Rd = RAddresses[2];

            Type_function = TypeofFunction(instruction);

            //Execute
            myRegistry.ReadWrite(Addr_Rs,Addr_Rt,NULL,NULL,0);
            $Rs = myRegistry.Read1;
            $Rt = myRegistry.Read2;

            // ALU operation
            if(Type_function == "addu") AluResult = myALU.ALUOperation(add_op, $Rs, $Rt);
            else if(Type_function == "and") AluResult = myALU.ALUOperation(and_op, $Rs, $Rt);
            else if(Type_function == "or") AluResult = myALU.ALUOperation(or_op, $Rs, $Rt);
            else if(Type_function == "nor") AluResult = myALU.ALUOperation(nor_op, $Rs, $Rt);
            else if(Type_function == "subu") AluResult = myALU.ALUOperation(subu_op, $Rs, $Rt);

            myRegistry.ReadWrite(NULL, NULL, Addr_Rd, AluResult, 1); //Writing result to write register
            programCounter = calc_PC(programCounter);
        }

        //Memory Write/Read
        else if(Type_instruction == "lw" || Type_instruction == "sw" || Type_instruction == "addiu" || Type_instruction == "beq")
        {
            //cout << "Instruction is " << instruction << endl;
            IAddressess = divInstruction_I(instruction);
            bitset<5> RsIAddr(IAddressess[0]);
            //cout << "RS" << RsIAddr << endl;
            bitset<5> RtIAddr(IAddressess[1]);
            bitset<16> ImmIAddr(IAddressess[2]);

            myRegistry.ReadWrite(RsIAddr, NULL, NULL, NULL, 0);

            $Rs = myRegistry.Read1;
            string finalImmStr; //complete Imm

            if (ImmIAddr.to_string().at(0) == '0') {
                //cout << "Zero Triggered" << endl;
                finalImmStr = "0000000000000000" + ImmIAddr.to_string();
            }
            else if (ImmIAddr.to_string().at(0) == '1') {
                //cout << "Check triggered" << endl;
                finalImmStr = "1111111111111111" + ImmIAddr.to_string();
            }
            bitset<32> finalImm(finalImmStr);

            AluResult = myALU.ALUOperation(add_op, $Rs, finalImm); //find effective address of memory

            if(Type_instruction == "lw") {
                bitset<32> operand1 = myMemory.MemoryAccess(AluResult, NULL, 1, 0); //get value from Data Memory

                myRegistry.ReadWrite(NULL, NULL, RtIAddr, operand1, 1); //load value into the register
                programCounter = calc_PC(programCounter);
            }

            else if(Type_instruction == "sw"){
                myRegistry.ReadWrite(RtIAddr,NULL,NULL,NULL,0); // get value from register
                $Rd = myRegistry.Read1;

                bitset<32> useless = myMemory.MemoryAccess(AluResult,$Rd,0,1); //store value to data memory
                //cout << "Memory Access" << useless.to_string() << endl;
                programCounter = calc_PC(programCounter);
            }

            else if(Type_instruction == "addiu"){
                AluResult = myALU.ALUOperation(add_op, $Rs, finalImm);
                myRegistry.ReadWrite(NULL,NULL,Addr_Rt,AluResult,1);
                programCounter = calc_PC(programCounter);
            }

            else if(Type_instruction == "beq"){
                myRegistry.ReadWrite(NULL, RtIAddr, NULL, NULL, 0);
                $Rt = myRegistry.Read2;
                if($Rs.to_ulong() == $Rt.to_ulong()){
                    //jump according to logic
                    finalImmStr = (finalImm<<2).to_string();
                    //finalImmStr = finalImm.to_string().substr(2,31)+"00";

                    bitset<32> signExtImm(finalImmStr);

                    counter = programCounter.to_ulong();

                    counter = counter + signExtImm.to_ulong();

                    bitset<32> pc(counter);
                    programCounter = calc_PC(pc);
                }
                else {
                    programCounter = calc_PC(programCounter);
                }
            }
        }
        else if (Type_instruction == "J"){
            bitset<26> JAddress(instruction.to_string().substr(6,26));
            counter = programCounter.to_ulong();
            counter = counter + 4;
            bitset<32> pc(counter);
            programCounter = pc;

            bitset<4> pc4MSB(programCounter.to_string().substr(0,4));
            string pcStr = pc4MSB.to_string() + JAddress.to_string() + "00";
            bitset<32> newPC(pcStr);
            programCounter = newPC;
        }
        myRegistry.outputRegistry(); // dump RF;
    }
    myMemory.OutputDataMem(); // dump data mem

    return 0;

}