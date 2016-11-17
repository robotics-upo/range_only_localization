#ifndef __SWARM_BEE_HPP__
#define __SWARM_BEE_HPP__

#include <termios.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <vector>

#define SWARM_BEE_GIVEUP_TIME 5			// Time in second to give up ranging a device without success
#define SWARM_BEE_RESCUE_TIME 10		// Time in second to try ranging again a giveup node

class SwarmBeeDev 
{
public:

	//!Nanotron range data
	struct RangeData 
	{
		unsigned int emitterId;		// Unique emitter node ID
		unsigned int receiverId;	// Unique recever node ID
		float range;				// Estimated range in meters
		float rssi;					// Received signal strength in 100%
		int status;					// Ranging status code:
									//		0 = success ranging result valid
									//		1 = ranging to own ID
									//		2 = ID out of range, no ACK
									//		3 = ranging unsuccessful, ACK OK, then timeout 5 = <TimeOut> has been reached
									//		6 = medium blocked (CSMA give up)		
		long long t_sec;    		// Epoch time in seconds
		long t_nsec;				// Nanoseconds into the Epoch		
	};

	//! Node information for ranging list
	struct nodeInfo
	{
		unsigned int nodeId;
		long long time;
		bool isActive;
		bool activeOnce;
	};

	//!Default constructor
	SwarmBeeDev(void)
	{
		m_portHandler = 0;
		m_baseId = -1;
	}

	//!Default destructor
	~SwarmBeeDev(void)	
	{
		finish();
	}

	/**
	 Intialize the serial port to the given values.
	 The function open the serial port in read-write mode

	 \param pDev Port of the serial device (e.g. '/dev/ttyUSB0')
	 
	 \return
	 - true: success
	 - false: error while open the device
	 */
	bool init(const std::string dev)
	{
		struct termios my_termios;

		// Make sure port is closed 
		if (m_portHandler > 0)
			close(m_portHandler);

		// Open the port in read-write mode 
		m_portHandler = open(dev.c_str(), O_RDWR | O_NOCTTY);
		if (m_portHandler < 0)
			return false;

		/* Get the port attributes and flush all data on queues*/
		tcgetattr(m_portHandler, &my_termios);
		tcflush(m_portHandler, TCIOFLUSH);

		/* Setup the communication */
		my_termios.c_iflag &= ~(BRKINT | IGNPAR | PARMRK | INPCK | ISTRIP | IXON
				| INLCR | IGNCR | ICRNL);
		my_termios.c_iflag |= IGNBRK | IXOFF;
		my_termios.c_oflag &= ~(OPOST);
		my_termios.c_cflag |= CLOCAL | CREAD;
		my_termios.c_cflag &= ~PARENB;
		my_termios.c_cflag |= CS8;
		my_termios.c_cflag &= ~CSTOPB;
		my_termios.c_cflag &= ~CRTSCTS;
		my_termios.c_lflag &= ~(ECHO | ECHOE | ECHOK | ECHONL | ICANON | NOFLSH
				| TOSTOP | ISIG | IEXTEN);
		my_termios.c_cc[VMIN] = 0; //Each simple read call will be blocked until receive at least one byte or last 100ms

		//VTIME = Timeout. Is a character count ranging from 0 to 255 characters.
		//It is time measured in 0.1 second intervals, (0 to 25.5 seconds).
		//More information in http://www.unixwiz.net/techtips/termios-vmin-vtime.html

		my_termios.c_cc[VTIME] = 1;	//0 = No timeout for reading, 1 = 100 ms timeout
		cfsetispeed(&my_termios, B115200);
		cfsetospeed(&my_termios, B115200);
		tcsetattr(m_portHandler, TCSANOW, &my_termios);

		return true;
	}	

	//!Finish the serial communication (closes serial port)
	void finish(void)
	{
		if (m_portHandler > 0) 
		{
			close(m_portHandler);
			m_portHandler = 0;
			m_baseId = -1;
		}
	}
	
	/**
	 * Reads the base node ID (MAC of node connected to serial port)
	 *
	 * return= -6; Error writing port.
	 * return= -5; Error readind port.
	 * return= >0; Node id.
	 **/
	int readBaseId(void)
	{		
		int res;
		
		// Write command
		cleanInput();
		strcpy(m_writeBuff, "GNID\r\n");
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		sscanf(m_readBuff, "%x", &m_baseId);

		return (int)m_baseId;
	}
	
	/**
	 * Reset settings from EEPROM
	 *
	 * return= -6: Error writing port.
	 * return= -5: Error readind port.
	 * return= -2: Command failed
	 * return=  0: Success.
	 **/
	int resetToEEPROM(void)
	{	
		int res;
		
		// Write command 
		cleanInput();
		strcpy(m_writeBuff, "RSET\r\n");
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		sscanf(m_readBuff, "%d", &res);
		if(res == 0)
			return 0;
		else
			return -2;		
	}
	
	/**
	 * Reset node to factory settings
	 *
	 * return= -6: Error writing port.
	 * return= -5: Error readind port.
	 * return= -2: Command failed
	 * return=  0: Success.
	 **/
	int resetToFactory(void)
	{	
		int res;
		
		// Write command 
		cleanInput();
		strcpy(m_writeBuff, "SFAC\r\n");
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		sscanf(m_readBuff, "%d", &res);
		if(res == 0)
			return 0;
		else
			return -2;		
	}
	
	/**
	 * Set power management
	 *
	 * \param val true for power saving ON, false for OFF
	 * 
	 * return= -6: Error writing port.
	 * return= -5: Error readind port.
	 * return= -2: Command failed
	 * return=  0: Success.
	 **/
	int setPowerManagement(bool val)
	{	
		int res;
		
		// Write command 
		cleanInput();
		if(val)
			strcpy(m_writeBuff, "SPSA 1\r\n");
		else
			strcpy(m_writeBuff, "SPSA 0\r\n");
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		sscanf(m_readBuff, "%d", &res);
		if( (res == 0 && !val ) || (res==1 && val) )
			return 0;
		else
			return -2;		
	}
	
	/**
	 * Set power management
	 *
	 * \param power Transmission power value from 0 to 100 
	 * 
	 * return= -6: Error writing port.
	 * return= -5: Error readind port.
	 * return= -2: Command failed
	 * return=  0: Success.
	 **/
	int setTransmissionPower(float power)
	{	
		int res, val;
		
		// Compute power in range 0 to 63
		val = (int)(power*63.0/100.0);
		if(val > 63)
			val = 63;
		if(val < 0)
			val = 0;
		
		// Write command 
		cleanInput();
		sprintf(m_writeBuff, "STXP %02d\r\n", val);
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		sscanf(m_readBuff, "%d", &res);
		if( res == val)
			return 0;
		else
			return -2;		
	}

	/**
	 * Compute distance to teh given node. 
	 * 
	 * \param nodeId The node to be requested to range.
	 * \param data Pointer to the structure where range information will be stored.
	 *
	 * return= -6: Error writing port.
	 * return= -5: Error reading port.
	 * return= -2: Command failed
	 * return=  0: Success.
	 **/
	int rangeTo(unsigned int nodeId, RangeData& data)
	{
		int res, dist, rssi, status;
		struct timespec stamp;
		
		// Write command 
		cleanInput();
		sprintf(m_writeBuff, "RATO 0 %012x\r\n", nodeId);
		res = writeCommand();
		if(res != 0)
			return res;

		// Read response
		clock_gettime(CLOCK_REALTIME, &stamp);
		res = readResponseSingle();
		if(res != 0)
			return res;

		// Parse response string 
		res = sscanf(m_readBuff, "%d %d %d", &status, &dist, &rssi);
		if(res == 3)
		{
			data.emitterId = m_baseId;
			data.receiverId = nodeId;
			data.range = (float)dist/100.0;
			data.rssi = (rssi+128.0)*1.075;
			data.status = status;
			data.t_sec = stamp.tv_sec;
			data.t_nsec = stamp.tv_nsec;
			if(data.rssi > 100)
				data.rssi = 100;
			if(data.rssi < 0)
				data.rssi = 0;
			if(data.range > 200)
			{
				data.status = 2;
				data.range = 0;
			}
			if(data.range < 0)
			{
				data.status = 2;
				data.range = 0;
			}
			return 0;
		}
		else
			return -2;		
	}
	
	/**
	 * Set the list of nodes to range. 
	 * 
	 * \param firstId First node in the list.
	 * \param lastId Last node in the list.
	 **/
	void setRangeList(unsigned int firstId, unsigned int lastId)
	{
		struct timespec time;
				
		// Clear list of nodes
		list.clear();
		list.resize(lastId-firstId+1);
		
		// Fill up node list
		clock_gettime(CLOCK_REALTIME, &time);
		for(int i=0; i<(int)list.size(); i++)
		{
			list[i].nodeId = i+firstId;
			list[i].time = time.tv_sec;
			list[i].isActive = true;
			list[i].activeOnce = false;
		}
		listIndex = 0;
	}
	
	int rangeToList(RangeData& data)
	{
		int i, res;
		struct timespec time;
		
		do
		{
			data.status = 2;
			if(list[listIndex].isActive)
			{
				// Check activeOnce flag
				if(list[listIndex].activeOnce)
				{
					list[listIndex].activeOnce = false;
					list[listIndex].isActive = false;
				}
						
				// Range to node
				res = rangeTo(list[listIndex].nodeId, data);
				
				// Check result
				if(data.status == 0 && res == 0)
				{
					list[listIndex].isActive = true;
					list[listIndex].time = data.t_sec;
				}
				
				else // Check active flag 
				{					
					clock_gettime(CLOCK_REALTIME, &time);
					if(time.tv_sec-list[listIndex].time > SWARM_BEE_GIVEUP_TIME)
					{
						list[listIndex].isActive = false;
						printf("\t Node %012x: Disabled\n", list[listIndex].nodeId);
					}
				}
			}
			
			else // Rescue disabled nodes to give them one try
			{
				clock_gettime(CLOCK_REALTIME, &time);
				if(time.tv_sec-list[listIndex].time > SWARM_BEE_RESCUE_TIME)
				{
					list[listIndex].time = time.tv_sec;
					list[listIndex].isActive = true;
					list[listIndex].activeOnce = true;
					printf("\t Node %012x: Recovering\n", list[listIndex].nodeId);
				}
			}
			
			// Prepare next reading
			listIndex++;
			if(listIndex >= list.size())
				listIndex = 0;
			
		}while(data.status != 0);
		
		return 0;
	}

protected:


	/**
	 * Write a command to the node. Command string is in m_writeBuff
	 *	 
	 * \return
	 * - -6: Writing error
	 * -  0: Success
	 **/
	int writeCommand(void) 
	{
		int writtenData, res, size;

		writtenData = res = 0;
		size = strlen(m_writeBuff);
		while(writtenData < size) 
		{
			res = write(m_portHandler, m_writeBuff + writtenData, size-writtenData);
			if (res < 0)
				return -6;
			writtenData += res;
		}
		
		return 0;
	}
	
	/**
	 * Read a single line response from node. Read string is stored in m_readBuff
	 *
	 * \return
	 * - -5: Reading error
	 * -  0: Success
	 **/
	int readResponseSingle(void) 
	{
		int res, size;
		char c;
		
		// Synch with the beginning of the messsage
		c = 0;
		while (c != '=') 
		{
			if (read(m_portHandler, &c, 1) <= 0) 
				return -5;
		}
		
		// Read message contents
		c = 0;
		size = 0;
		while(c != '\n') 
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;

			if(res == 1) 
			{
				if(c == ',')
					c = ' ';
				m_readBuff[size++] = c;
			}
		}
		m_readBuff[size] = '\0';
		
		return 0; 
	}
	
	/**
	 * Read a multi line response from node. Read string is stored in m_readBuff
	 *
	 * \return
	 * - -5: Reading error
	 * -  0: Success
	 **/
	int readResponseMulti(void) 
	{
		int res, size, lines;
		char c;
		
		// Synch with the beginning of the messsage
		c = 0;
		while (c != '#') 
		{
			if (read(m_portHandler, &c, 1) <= 0) 
				return -5;
		}
		
		// Read the number of lines
		c = 0;
		size = 0;
		while(c != '\n') 
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;

			if(res == 1) 
				m_readBuff[size++] = c;
		}
		m_readBuff[size] = '\0';
		sscanf(m_readBuff, "%d", &lines);
		
		// Read message contents
		c = 0;
		size = 0;
		while(lines > 0) 
		{
			res = read(m_portHandler, &c, 1);
			if(res <= 0) 
				return -5;

			if(res == 1) 
			{
				if(c == ',')
					c = ' ';
				m_readBuff[size++] = c;
				if(c == '\n')
					lines--;
			}
		}
		m_readBuff[size] = '\0';
		
		return 0; 
	}
	
	void cleanInput(void)
	{
		tcflush(m_portHandler, TCIFLUSH);
	}

	//!Serial port handler
	int m_portHandler;

	//!Reading buffer
	char m_readBuff[256];
	
	//!Command buffer
	char m_writeBuff[256];

	//!Base Id
	unsigned int m_baseId;
	
	//! List of nodes to range
	std::vector<nodeInfo> list;
	int listIndex;
};

#endif

