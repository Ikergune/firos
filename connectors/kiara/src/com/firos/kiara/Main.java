package com.firos.kiara;

import com.firos.kiara.connector.Connector;


public class Main {
	
	public static String KIARA_IP = "239.255.0.1";
	public static int KIARA_PORT = 7400;
	
	private static void loadArgs(String[] args){
		for (int i = 0; i < args.length; i++) {
			if(args[i].equals("-a")){
				i++;
				KIARA_IP = args[i];
			}
			if(args[i].equals("-p")){
				i++;
				KIARA_PORT = Integer.parseInt(args[i]);
			}
		}		
	}

	public static void main(String[] args) {
		loadArgs(args);
	    Main app = new Main();
	    
		Connector connector = Connector.getInstance();
		Runtime.getRuntime().addShutdownHook(new Thread() {
            public void run() {
            	System.out.println("EXIT JAVA");
            	connector.stop();
            }
        });
	}

}
