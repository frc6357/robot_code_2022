package frc.robot.subsystems;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.charset.StandardCharsets;
import java.util.concurrent.TimeoutException;

import javax.swing.text.StyledEditorKit;

import frc.robot.Constants;
import frc.robot.utils.CRC;

public class SK22Vision extends SKSubsystemBase implements AutoCloseable {
    // destination ports are required, but source ports are optional
    final int odroidPort = 5005;
    // this will change depending on the network connected
    final String odroidIP = "";
    final int roborioPort = 5800;
    private String caughtException = "";
    ByteBuffer rDataBuffer = ByteBuffer.allocate(Constants.VisionConstants.UDP_PACKET_LENGTH);
    // not needed yet
    // byte[] sDataBuffer = new byte[1024];
    DatagramChannel sSocket;
    ByteArrayInputStream byteArrayInputStream;
    DataInputStream dataInputStream;
    byte[] versionID = new byte[1];
    byte[] packetLength = new byte[1];
    byte[] frameID = new byte[4];
    byte[] timestamp = new byte[4];
    byte[] distance = new byte[2];
    byte[] horiAngle = new byte[2];
    byte[] vertAngle = new byte[2];
    byte[] reserved = new byte[8];
    byte[] pythonChecksum = new byte[2];


    public SK22Vision()
    {
        try {
            sSocket = DatagramChannel.open();
            sSocket.configureBlocking(false);
            sSocket.socket().bind(new InetSocketAddress(roborioPort));
            
        } catch (Exception e) {
            //TODO: handle exception
            System.out.println(e.toString());
        }

        

        // memory allocated for packets to be read on the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

        // memory allocated for packets to be sent from the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

    }
    
    // returns the boolean that represents whether or not an exception was caught
    public String caughtException(String exceptionMessage)
    {
        return caughtException;
    }

    public boolean getPacket(DatagramChannel sSocket) 
    {
        boolean returnCode = false;
        try
        {
            // get the packet at the port defined by sSocket
            if (!(sSocket.receive(rDataBuffer) == null))
            {
                returnCode = true;
            }

        

        }
        catch (Exception e) 
        {
            caughtException = e.toString();
            System.out.println("Socket Excetion: " + caughtException);
            returnCode = false;
        }

        return returnCode;
    }

    public void sendPacket(byte[] sDataBuffer, DatagramSocket sSocket, String message)
    {
        try 
        {
            // converts string data to byte data and stores this in the Data Buffer to be sent
            sDataBuffer = message.getBytes();
            // takes byte data stored in sDataBuffer and sends it to the odroid's IP address at the specified port on the odroid
            DatagramPacket outputPacket = new DatagramPacket(sDataBuffer, sDataBuffer.length, InetAddress.getByName(odroidIP), odroidPort);
            sSocket.send(outputPacket); 
        }
        catch (IOException i)
        {
            System.out.println("IO Exception" + i.toString());
        }
    }

    public void getByteBuffer(byte[] inputBytes, ByteBuffer byteBuffer, int minIndex, int maxIndex)
    {
        int arrayIndex = 0;
        for (int i = minIndex; i<maxIndex; i++){
            inputBytes[arrayIndex] = byteBuffer.get(i);
            arrayIndex++;
        }
    }

    public int convertBytesInt(byte[] byteArray)
    {
        ByteBuffer byteArrayBuffer = ByteBuffer.wrap(byteArray);
        byteArrayBuffer.order(ByteOrder.LITTLE_ENDIAN);
        int dataByteArrayBuffer = byteArrayBuffer.getInt();
        return dataByteArrayBuffer;
    }

    public short convertBytesShort(byte[] byteArray)
    {
        ByteBuffer byteArrayBuffer = ByteBuffer.wrap(byteArray);
        byteArrayBuffer.order(ByteOrder.LITTLE_ENDIAN);
        short dataByteArrayBuffer = byteArrayBuffer.getShort();
        return dataByteArrayBuffer;
    }

    public char convertBytesChar(byte[] byteArray)
    {
        ByteBuffer byteArrayBuffer = ByteBuffer.wrap(byteArray);
        byteArrayBuffer.order(ByteOrder.LITTLE_ENDIAN);
        char dataByteArrayBuffer = byteArrayBuffer.getChar();
        return dataByteArrayBuffer;
    }
    
    public long convertBytesLong(byte[] byteArray)
    {
        ByteBuffer byteArrayBuffer = ByteBuffer.wrap(byteArray);
        byteArrayBuffer.order(ByteOrder.LITTLE_ENDIAN);
        long dataByteArrayBuffer = byteArrayBuffer.getLong();
        return dataByteArrayBuffer;
    }

    
    @Override
    public void periodic()
    {
        // reads packet data and print to terminal
        if(getPacket(sSocket))
        {
            // TODO: Move this to a function
            versionID[0] = rDataBuffer.get(0);
            packetLength[0] = rDataBuffer.get(1);
            getByteBuffer(frameID, rDataBuffer, 2, 6);
            getByteBuffer(timestamp, rDataBuffer, 6, 10);
            getByteBuffer(distance, rDataBuffer, 10, 12);
            getByteBuffer(horiAngle, rDataBuffer, 12, 14);
            getByteBuffer(vertAngle, rDataBuffer, 14, 16);
            getByteBuffer(reserved, rDataBuffer, 16, 24);
            getByteBuffer(pythonChecksum, rDataBuffer, 24, 26);
            char versionIDData = convertBytesChar(versionID);
            char packetLengthData = convertBytesChar(packetLength);
            int frameIDData = convertBytesInt(frameID);
            int distanceData = convertBytesInt(distance);
            short horiAngleData =  convertBytesShort(horiAngle);
            short vertAngleData = convertBytesShort(vertAngle);
            long reservedData = convertBytesShort(reserved);
            short pythonChecksumData = convertBytesShort(pythonChecksum);
        }

        /*  try {
            int inputStreamAvailable = packetData.available();
            System.out.println("packet data available: " + inputStreamAvailable);
            int distanceBytes = packetData.read(distance, 10, 2);
            
        } catch (IOException e) {
            // TODO Auto-generated catch block
            System.out.println("DataInputStream available(): " + e.toString());
        } */
        

        
        /*versionID = packetData.getChar(0);
        frameID = packetData.getInt(2);
        timestamp = packetData.getInt(6);
        distance = packetData.getShort(10);
        horiAngle = packetData.getShort(12);
        vertAngle = packetData.getShort(14);
        reserved = packetData.getLong(16);
        pythonChecksum = packetData.getShort(24); */

        // System.out.println(versionID);
        // System.out.println(packetLength);
        // System.out.println(frameID);
        // System.out.println(timestamp);
        // System.out.println(distance);
        // System.out.println(horiAngle);
        // System.out.println(vertAngle);s
        // System.out.println(reserved);
        // System.out.println(pythonChecksum);

    }   

    
    @Override
    public void initializeTestMode()
    {

    }

    @Override
    public void testModePeriodic()
    {

    }

    @Override
    public void enterTestMode()
    {

    }

    @Override
    public void close() throws Exception{

    }


}
