package frc.robot.subsystems;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;

public class SK22Vision extends SKSubsystemBase implements AutoCloseable {
    // destination ports are required, but source ports are optional
    final int odroidPort = 5005;
    // this will change depending on the network connected
    final String odroidIP = "";
    final int roborioPort = 5800;
    public DatagramSocket sSocket;
    byte[] rDataBuffer;
    byte[] sDataBuffer;

    private String caughtException = "";
    public SK22Vision()
    {
        
        try {
            // binds socket to specified port on computer
            sSocket = new DatagramSocket(roborioPort);

            // memory allocated for packets to be read on the computer
            // ideally the memory is not hardcoded, but dynamic based on the packet size
            rDataBuffer = new byte[1024];

            // memory allocated for packets to be sent from the computer
            // ideally the memory is not hardcoded, but dynamic based on the packet size
            byte[] sDataBuffer = new byte[1024];
        
        }

        catch (SocketException e) 
        {
            caughtException = e.toString();
        }
        
    }
    // returns the boolean that represents whether or not an exception was caught
    public String caughtException(String exceptionMessage)
    {
        return caughtException;
    }

    public String getPacket(DatagramSocket sSocket, byte[] rDataBuffer) 
    {
        try
        {

        // create Datagram packet with equal size to the 
        DatagramPacket inputPacket = new DatagramPacket(rDataBuffer, rDataBuffer.length);
        System.out.println("Waiting for message...");

        // get the packet at the port defined by sSocket
        sSocket.receive(inputPacket);
        
        // reads data from packet to string
        String rData = new String(inputPacket.getData());
        
        // refreshes memory to empty
        this.rDataBuffer = new byte[1024];

        return rData;

        }
        catch (SocketException e) 
        {
            caughtException = e.toString();
            return caughtException;
        }

        catch (IOException i)
        {
            caughtException = i.toString();
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
            System.out.println(i.toString());
        }
    }


    
    @Override
    public void periodic()
    {
        try (SK22Vision m_Sk22Vision = new SK22Vision()) {
            // reads packet data and print to terminal
            System.out.println(getPacket(m_Sk22Vision.sSocket, m_Sk22Vision.rDataBuffer));
            // close the socket after reading data
            m_Sk22Vision.sSocket.close();
        } catch (Exception e) {
            // print exception message to log
            System.out.println(e.toString());
        }
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
