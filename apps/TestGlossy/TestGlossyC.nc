#define GLOSSY_INIT_BACKOFF 5*1024L
#define GLOSSY_CONGEST_BACKOFF 2*1024L

module TestGlossyC {
  uses {
    interface Boot;
    interface CC2420Power;
    interface StdControl as SubControl;
    interface Receive;
    interface Resource;
    interface Leds;

    interface CC2420Transmit as Send;
    interface RadioBackoff;
    interface Random;

    interface Timer<TMicro> as Timer;
  }
} implementation {
  task void null_task();
  task void start_radio();

  message_t m_msg;
  uint16_t m_dsn = 0;

  uint16_t snd_counter = 200;
  uint16_t rec_counter = 0;

  uint16_t lastest_dsn = 0xffff;

  event void Boot.booted() {
    // post null_task();
    call CC2420Power.startVReg();
    // call Timer.startOneShot(4L*1048576L);
    call Timer.startPeriodic(4L*1048576L);
  }

  async event void CC2420Power.startVRegDone() {
    call Resource.request();
  }

  async event void CC2420Power.startOscillatorDone() {
    post start_radio();
  }

  event void Resource.granted() {
    call CC2420Power.startOscillator();
  }

  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    cc2420_header_t* m_ptr_header = (cc2420_header_t*)((uint8_t*)msg->data - sizeof(cc2420_header_t));
    call Leds.led2Toggle();
    if (lastest_dsn != m_ptr_header->dsn) {
      rec_counter++;
      // pr("%d\n", rec_counter);
      pr("%d\n", m_ptr_header->dsn);
    }
    lastest_dsn = m_ptr_header->dsn;
    return msg;
  }

  async event void Send.sendDone(message_t* msg, error_t err) {
    m_dsn++;
    snd_counter--;
    if (snd_counter == 0) {
      call Timer.stop();
      pr("Packets Received: %d\n", rec_counter);
    }
    if (((cc2420_metadata_t*)(msg->metadata))->ack) {
      call Leds.led1Toggle();
    }
  }

  async event void RadioBackoff.requestInitialBackoff(message_t* msg) {
    call RadioBackoff.setInitialBackoff(call Random.rand16() % GLOSSY_INIT_BACKOFF);
  }

  async event void RadioBackoff.requestCongestionBackoff(message_t* msg) {
    call RadioBackoff.setInitialBackoff(call Random.rand16() % GLOSSY_CONGEST_BACKOFF);
  }

  async event void RadioBackoff.requestCca(message_t* msg) {
  }

  event void Timer.fired() {
    uint8_t i = 0;
    message_t* m_ptr_msg = &m_msg;
    cc2420_header_t* m_ptr_header = (cc2420_header_t*)((uint8_t*)m_ptr_msg->data - sizeof(cc2420_header_t));
    uint8_t* m_ptr_data = (uint8_t*)m_ptr_msg->data;

    memset((uint8_t*)m_ptr_msg, 0, sizeof(message_t));

    m_ptr_header->length = DATA_LENGTH + CC2420_SIZE;
    m_ptr_header->fcf |= ( 1 << IEEE154_FCF_INTRAPAN ) 
                 | ( IEEE154_ADDR_SHORT << IEEE154_FCF_DEST_ADDR_MODE )
                 | ( IEEE154_ADDR_SHORT << IEEE154_FCF_SRC_ADDR_MODE ) ;
    // m_ptr_header->fcf |= (1 << IEEE154_FCF_ACK_REQ);
    m_ptr_header->dsn = m_dsn;
    m_ptr_header->destpan = 0;
    m_ptr_header->dest = 0xFFFF;
    m_ptr_header->src = TOS_NODE_ID;

    for (i = 0; i < 30; i++) {
      m_ptr_data[i] = i;
    }

    call Send.send(m_ptr_msg, FALSE); 
    call Leds.led0Toggle();
  }

  task void null_task() {
    post null_task();
  }
 
  task void start_radio() {
    call CC2420Power.rxOn();
    call Resource.release();
    call SubControl.start();
  }
}
