package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.charset.Charset;
import javax.xml.crypto.Data;
import frc.robot.Constants;

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

    public String getPacket(DatagramChannel sSocket, ByteBuffer rDataBuffer) 
    {
        try
        {
        // get the packet at the port defined by sSocket
        sSocket.receive(rDataBuffer);
        System.out.println("Waiting for message...");

        
        // reads data from packet to string
        if (rDataBuffer.hasArray())
        {
            String rData = new String(rDataBuffer.array(), Charset.defaultCharset());
            return rData;
        }
        else
        {
            String rData = new String("rDataBuffer no Array");
            return rData;
        }

        

        }
        catch (SocketException e) 
        {
            caughtException = e.toString();
            System.out.println("Socket Excetion: " + caughtException);
            return caughtException;
        }

        catch (IOException i)
        {
            caughtException = i.toString();
            System.out.println("IO Excetion: " + caughtException);
            return caughtException;
        }

        
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


    
    @Override
    public void periodic()
    {
        // reads packet data and print to terminal
        String packetData = getPacket(sSocket, rDataBuffer);
        // print packet data to log
        System.out.println(packetData);
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
