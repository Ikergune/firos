package com.firos.kiara.connector;

import com.firos.kiara.Main;
import com.firos.kiara.interfaces.Firos;

import py4j.GatewayServer;

//Kiara Depndencies
import org.fiware.kiara.ps.rtps.RTPSDomain;

import org.fiware.kiara.ps.rtps.common.Locator;
import org.fiware.kiara.ps.rtps.common.ReliabilityKind;
import org.fiware.kiara.ps.rtps.common.EncapsulationKind;

import org.fiware.kiara.ps.rtps.history.CacheChange;
import org.fiware.kiara.ps.rtps.history.WriterHistoryCache;

import org.fiware.kiara.ps.rtps.attributes.WriterAttributes;
import org.fiware.kiara.ps.rtps.attributes.RemoteReaderAttributes;
import org.fiware.kiara.ps.rtps.attributes.HistoryCacheAttributes;
import org.fiware.kiara.ps.rtps.attributes.RTPSParticipantAttributes;

import org.fiware.kiara.ps.rtps.messages.common.types.ChangeKind;
import org.fiware.kiara.ps.rtps.messages.elements.InstanceHandle;
//import org.fiware.kiara.ps.rtps.messages.elements.SerializedPayload;

import org.fiware.kiara.ps.rtps.writer.RTPSWriter;
import org.fiware.kiara.ps.rtps.participant.RTPSParticipant;

import org.fiware.kiara.serialization.impl.Serializable;

public class Connector {
	
	private static Connector connector = new Connector();
	private GatewayServer server; 
	
	private Firos firos;
	private RTPSWriter writer = null;
    private WriterHistoryCache w_history  = null;
    private RTPSParticipantAttributes pparam = new RTPSParticipantAttributes();
	
	public Connector(){
		//	GateWay methods
		server = new GatewayServer(this);
	    server.start();
	    
	    // Kiara Methods
        pparam.builtinAtt.useSimplePDP = false;
        pparam.builtinAtt.useWriterLP = false;

        RTPSParticipant participant = RTPSDomain.createParticipant(pparam, null);

        if(participant == null)
        {
            System.out.println("ERROR creating participant");
            return;
        }

        HistoryCacheAttributes hatt = new HistoryCacheAttributes();
        hatt.payloadMaxSize = 500;

        w_history = new WriterHistoryCache(hatt);

        WriterAttributes watt = new WriterAttributes();
        watt.endpointAtt.reliabilityKind = ReliabilityKind.BEST_EFFORT;

        writer = RTPSDomain.createRTPSWriter(participant, watt, w_history, null);

        if(writer == null)
        {
            System.out.println("ERROR creating writer");
            return;
        }

        RemoteReaderAttributes ratt = new RemoteReaderAttributes();
        Locator loc = new Locator();
        loc.setIPv4Address(Main.KIARA_IP);
        loc.setPort(Main.KIARA_PORT);

        ratt.endpoint.multicastLocatorList.pushBack(loc);
        writer.matchedReaderAdd(ratt);
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
	
	public void sendMessage(String data){
		System.out.println("SEND");
		System.out.println(data);
		CacheChange change = writer.newChange(ChangeKind.ALIVE, new InstanceHandle());
        Message message = new Message();
        message.setStr(data);

        change.getSerializedPayload().setEncapsulationKind(EncapsulationKind.CDR_LE);
        change.getSerializedPayload().setData((Serializable)message);
        change.getSerializedPayload().setLength((short)(4 + 255));
        w_history.addChange(change);
	}
	
	public void stop(){
		this.server.shutdown();
	}
}
