package com.firos.kiara;

import com.firos.kiara.connector.Connector;

import py4j.GatewayServer;

public class Main {
	
	public static String KIARA_IP = "239.255.0.1";
	public static int KIARA_PORT = 7400;
	
	private static void loadArgs(String[] args){
		for (int i = 0; i < args.length; i++) {
			if(args[i] == "-a"){
				i++;
				KIARA_IP = args[i];
			}
			if(args[i] == "-p"){
				i++;
				KIARA_PORT = Integer.parseInt(args[i]);
			}
		}		
	}

	public static void main(String[] args) {
		loadArgs(args);
	    Main app = new Main();
	    // app is now the gateway.entry_point
	    GatewayServer server = new GatewayServer(app);
	    server.start();
		// TODO Auto-generated method stub
		Connector connector = Connector.getInstance();
	}

}
