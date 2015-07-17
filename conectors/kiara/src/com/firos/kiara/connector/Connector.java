package com.firos.kiara.connector;

import com.firos.kiara.interfaces.Firos;

import py4j.GatewayServer;

public class Connector {
	
	private static Connector connector = new Connector();
	private GatewayServer server; 
	private Firos firos;
	
	public Connector(){
		server = new GatewayServer(this);
	    server.start();
	}
	
	public static Connector getInstance() {
		return connector;
    }
	
	public void setFiros(Firos firos) {
		this.firos = firos;
	}
	
	public void onData(String data){
		System.out.println("RECEIVED FIROS");
		this.firos.onData(data);
	}
	
	public void sendMessage(String message){
		System.out.println("SEND");
		System.out.println(message);
	}
	
	public void stop(){
		this.server.shutdown();
	}
}
