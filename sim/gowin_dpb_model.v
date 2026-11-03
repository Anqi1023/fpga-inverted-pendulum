`timescale 1ns/1ps
// Test-only functional model for the exact DPB configuration used by the trace.
// Gowin Designer supplies the real primitive. Do NOT add this file to the GPRJ.
// x16 addresses use ADA/ADB[13:4]; mode00 holds DO while the port is writing.
module DPB #(
    parameter READ_MODE0=1'b0,READ_MODE1=1'b0,
    parameter [1:0] WRITE_MODE0=2'b00,WRITE_MODE1=2'b00,
    parameter integer BIT_WIDTH_0=16,BIT_WIDTH_1=16,
    parameter [2:0] BLK_SEL_0=3'b000,BLK_SEL_1=3'b000,
    parameter RESET_MODE="SYNC"
)(
    output reg [15:0] DOA,DOB,
    input wire CLKA,CEA,OCEA,RESETA,WREA,
    input wire CLKB,CEB,OCEB,RESETB,WREB,
    input wire [13:0] ADA,ADB,
    input wire [2:0] BLKSELA,BLKSELB,
    input wire [15:0] DIA,DIB
);
    reg [15:0] memory [0:1023];
    always @(posedge CLKA)if(CEA && BLKSELA==BLK_SEL_0)begin
        if(RESETA)DOA<=0;
        else if(WREA)begin
            if(ADA[0])memory[ADA[13:4]][7:0]<=DIA[7:0];
            if(ADA[1])memory[ADA[13:4]][15:8]<=DIA[15:8];
        end
        else DOA<=memory[ADA[13:4]];
    end
    always @(posedge CLKB)if(CEB && BLKSELB==BLK_SEL_1)begin
        if(RESETB)DOB<=0;
        else if(WREB)begin
            if(ADB[0])memory[ADB[13:4]][7:0]<=DIB[7:0];
            if(ADB[1])memory[ADB[13:4]][15:8]<=DIB[15:8];
        end
        else DOB<=memory[ADB[13:4]];
    end
    // synthesis translate_off
    initial if(READ_MODE0!=0 || READ_MODE1!=0 || WRITE_MODE0!=0 || WRITE_MODE1!=0 ||
        BIT_WIDTH_0!=16 || BIT_WIDTH_1!=16 || RESET_MODE!="SYNC")
        $fatal(1,"Trace DPB model accepts only the supported x16 normal-mode configuration");
    always @(posedge CLKA)if(CEA && (ADA[3:2]!=0 || (WREA && ADA[1:0]!=2'b11)))
        $fatal(1,"DPB x16 port A address/byte enables incorrect");
    always @(posedge CLKB)if(CEB && (ADB[3:2]!=0 || (WREB && ADB[1:0]!=2'b11)))
        $fatal(1,"DPB x16 port B address/byte enables incorrect");
    // synthesis translate_on
endmodule
