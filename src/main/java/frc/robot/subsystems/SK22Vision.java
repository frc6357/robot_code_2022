package frc.robot.subsystems;

import java.io.ByteArrayInputStream;
import java.io.DataInputStream;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.util.zip.CRC32;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** The Datagram Class defines and unpacks the contents of the packets sent by the Odroid-XU4 */
class Datagram
{
    int versionID;
    int packetLength;
    long frameID;
    long timestamp;
    double distance;
    int adjVal;
    float horiAngle;
    float vertAngle;
    long reserved;
    long checksum;
    boolean validPacket = false;

    Datagram(ByteBuffer byteBuffer)
    {
        byteBuffer.position(0);

        byte versionIDSigned = byteBuffer.get();
        versionID = Byte.toUnsignedInt(versionIDSigned);

        byte packetLengthSigned = byteBuffer.get();
        packetLength =  Byte.toUnsignedInt(packetLengthSigned);

        int frameIDSigned = byteBuffer.getInt();
        frameID = Integer.toUnsignedLong(frameIDSigned);

        int timestampSigned = byteBuffer.getInt();
        timestamp = Integer.toUnsignedLong(timestampSigned);

        short distanceSigned = byteBuffer.getShort();
        int distanceInt = Short.toUnsignedInt(distanceSigned);
        distance = distanceInt/(Math.pow(10, adjVal));

        byte adjValSigned = byteBuffer.get();
        adjVal = Byte.toUnsignedInt(adjValSigned);

        short shortHoriAngle = byteBuffer.getShort();
        horiAngle = shortHoriAngle/100;

        short shortVertAngle = byteBuffer.getShort();
        vertAngle = shortVertAngle/100;

        reserved = byteBuffer.getLong();
        checksum = byteBuffer.getInt();

        CRC32 crc = new CRC32();
        crc.update(byteBuffer);
        validPacket = (crc.getValue() == 0) ? true : false;
    }
}

/** This class retrieves packets sent by the Odroid-XU4 at port 5800 and unpacks the 
 *  data using the Datagram class above, within the periodic */
public class SK22Vision extends SKSubsystemBase implements AutoCloseable 
{
    // destination ports are required, but source ports are optional
    final int odroidPort = Constants.VisionConstants.ODROID_PORT;
    
    final int roborioPort = Constants.VisionConstants.ROBORIO_PORT;
    
    private String caughtException = "";
    
    ByteBuffer rDataBuffer = ByteBuffer.allocate(Constants.VisionConstants.UDP_PACKET_LENGTH)
                            .order(ByteOrder.LITTLE_ENDIAN);
    ByteBuffer packetBuffer = ByteBuffer.allocate(Constants.VisionConstants.UDP_PACKET_LENGTH)
                            .order(ByteOrder.LITTLE_ENDIAN);
    
    DatagramChannel sSocket;
    ByteArrayInputStream byteArrayInputStream;
    DataInputStream dataInputStream;

    // The packet containing the angle and distance values
    Datagram packetDatagram;

    /** This constructor initializes the port 5800 on the roborio for reading UDP packets. 
     *  Disables blocking so it can be used in periodic */
    public SK22Vision()
    {
        try 
        {
            sSocket = DatagramChannel.open();
            sSocket.configureBlocking(false);
            sSocket.socket().bind(new InetSocketAddress(roborioPort));
            
        } 
        
        catch (Exception e) 
        {
            //TODO: handle exception
            System.out.println(e.toString());
        }

        

        // memory allocated for packets to be read on the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

        // memory allocated for packets to be sent from the computer
        // ideally the memory is not hardcoded, but dynamic based on the packet size

    }
    
    /** returns the boolean that represents whether or not an exception was caught 
     *  @param exceptionMessage
     *          An exception .toString() within a try/catch block 
     *  
     *  @return Returns the string representation of an exception
     * */ 
    
    public String caughtException(String exceptionMessage)
    {
        return caughtException;
    }

    /**  Receive the latest available UDP packet if any is available. Return
    * true if a packet has been read, false if none is available.
    *
    * This method discards all but the latest received packet on the 
    * socket.
    * @param sSocket
    *           A non-blocking DatagramChannel object that is used to initialize the 
    *           packet receiving port on the roborio
    * 
    * @return   True/False if a packet was received at the DatagramChannel on the specified port
    */
    public boolean getPacket(DatagramChannel sSocket) 
    {
        boolean receivedOne = false;
        SocketAddress sockRet;
        

        do
        {
            rDataBuffer.position(0);

            try
            {
                // get the packet at the port defined by sSocket
                sockRet = sSocket.receive(rDataBuffer);
                if (sockRet != null)
                {
                    // We got a packet! Save it into a local buffer
                    // for later.
                    receivedOne = true;
                    packetBuffer = rDataBuffer.duplicate();
                    packetBuffer.order(ByteOrder.LITTLE_ENDIAN);
                }
            }
            catch (Exception e) 
            {
                caughtException = e.toString();
                System.out.println("Socket Excetion: " + caughtException);
                sockRet = null;
            }
        } while (sockRet != null);

        return receivedOne;
    }
    
    @Override
    public void periodic()
    {
        boolean packetRecieved = getPacket(sSocket);
        // check if packet is valid
        if (packetRecieved)
        {
            packetDatagram = new Datagram(packetBuffer);
            if (packetDatagram.validPacket)
            {
                SmartDashboard.putNumber("Distance",            packetDatagram.distance);
                SmartDashboard.putNumber("Horizontal Angle",    packetDatagram.horiAngle);
                SmartDashboard.putNumber("Vertical Angle",      packetDatagram.vertAngle);
            }
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
    public void close() throws Exception 
    {

    }


}
