import net.tinyos.util.*;
import net.tinyos.packet.*;

import java.io.*;
import java.util.*;

public class FastPrintf {
	public static void main(String args[]) throws IOException {
		String source = null;
		ByteSource io;
		Boolean u8_print = false;
		Boolean u16_print = false;
		Boolean int_print = false;
		int u8_len = 0;
		int u16_len = 0;
		int int_len = 0;
		if (args.length == 2 && args[0].equals("-comm")) {
			source = args[1];
		} else if (args.length > 0) {
	    		System.exit(2);
		}
		io = makeByteSource(source);
		if (io == null) {
	    		System.exit(2);
		}

		try {
			io.open();
			while(true) {
				byte b = io.readByte();
				int low;
				int high;
				int word;
				short value;
				if (b == 0x22 && !u8_print) {
					u8_print = true;
				}else if (b == 0x77 && !u16_print) {
					u16_print = true;
				}else if (b == 0x44 && !int_print) {
					int_print = true;
                                }else if (u8_print && u8_len == 0) {
					u8_len = b;
				}else if (u16_print && u16_len == 0) {
					u16_len = b;
                                }else if (int_print && int_len == 0) {
					int_len = b;
				}else if (u8_print && u8_len != 0) {
                                        low = b & 0x000000ff;
					System.out.print(low + " ");
					u8_len--;
					if (u8_len == 0) {
						System.out.println();
						System.out.flush();
						u8_print = false;
					}
				}else if (u16_print && u16_len != 0) {
					high = b & 0x000000ff;
					low  = io.readByte() & 0x000000ff;
					word = high * 256 + low;
					System.out.print(word + " ");
					u16_len--;
					if (u16_len == 0) {
						System.out.println();
						System.out.flush();
						u16_print = false;
					}
				}else if (int_print && int_len != 0) {
					high = b & 0x000000ff;
					low  = io.readByte() & 0x000000ff;
					value = (short)(high * 256 + low);
					System.out.print(value + " ");
					int_len--;
					if (int_len == 0) {
						System.out.println();
						System.out.flush();
						int_print = false;
					}
                                }
			}
		} catch (IOException e) { System.out.println("error " + e); }

		io.close();
	}

	public static ByteSource makeByteSource(String args) {
		if (args == null)
			return null;

		String[] array = args.split("@");
		String type = array[0];
		String portbaud = array[1];

		if (!type.equals("serial"))
			return null;

		return makeArgsSerial(portbaud);
	}

	public static ByteSource makeArgsSerial(String args) {
		String[] array = args.split(":");
		String port = array[0];
		int baudrate = 115200;
		System.out.println(port+":"+baudrate);
		return new SerialByteSource(port, baudrate);
	}
}
