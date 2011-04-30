/** @file
 * @brief Include template code for uart.c.
 *
 * The N_(p,s) macro must be defined before including.
 * It is automatically undefined at the end of this file.
 */

#define UARTN(s) N_(UART,s)
#define uartN(s) N_(uart,s)
#define SIG_UARTN(s)  N_(SIG_USART,_##s)


typedef struct {
  uint8_t *head;
  uint8_t *tail;
  uint8_t data[UARTN(_RX_BUF_SIZE)];
} UARTN(RxBuf);

typedef struct {
  uint8_t *head;
  uint8_t *tail;
  uint8_t data[UARTN(_TX_BUF_SIZE)];
} UARTN(TxBuf);

/// Fifo for received data.
static UARTN(RxBuf) uartN(_rx_buf);
/// Fifo for sent data.
static UARTN(TxBuf) uartN(_tx_buf);


/// Initialize the UART.
static void uartN(_init)(void)
{
  fifobuf_init((FifoBuf *)&uartN(_rx_buf));
  fifobuf_init((FifoBuf *)&uartN(_tx_buf));

#define UART_UBRR_VAL \
  (uint16_t)((float)(F_CPU) / (( (UARTN(_DOUBLE_SPEED)) ? 8 : 16 )*(UARTN(_BAUDRATE))) - 1 )
  N_(UBRR,H) = UART_UBRR_VAL>>8;
  N_(UBRR,L) = UART_UBRR_VAL;
#undef UART_UBRR_VAL
  N_(UCSR,A) = (UARTN(_DOUBLE_SPEED)) ? (1<<U2X) : 0;
  N_(UCSR,B) = (1<<RXEN) | (1<<TXEN) | (1<<RXCIE);
  N_(UCSR,C) = (3<<UCSZ0);
}


/** @brief Send next waiting char, if any.
 * @note Must be called with IRQ locked.
 */
static void uartN(_send_fifo_char)(void)
{
  if( FIFOBUF_ISEMPTY( &uartN(_tx_buf) ) ) {
    N_(UCSR,B) &= ~(1<<UDRIE);
  } else {
    FIFOBUF_POP( &uartN(_tx_buf), N_(UDR,) );
    N_(UCSR,B) |= (1<<UDRIE);
  }
}



int uartN(_recv)(void)
{
  int ret;
  while( (ret = uartN(_recv_nowait)()) < 0 ) ;
  return ret;
}


int uartN(_recv_nowait)(void)
{
  uint8_t flags;
  IRQ_LOCK(flags);

  if( FIFOBUF_ISEMPTY( &uartN(_rx_buf) ) ) {
    IRQ_UNLOCK(flags);
    return -1;
  }

  uint8_t v;
  FIFOBUF_POP( &uartN(_rx_buf), v );

  IRQ_UNLOCK(flags);
  return v;
}


int uartN(_send)(uint8_t v)
{
  if( uartN(_send_nowait)(v) < 0 ) {
    if( GLOBAL_IRQ_ARE_MASKED() && (N_(UCSR,B) & (1<<RXCIE)) ) {
      // poll if IRQ are locked
      while( !(N_(UCSR,A) & (1<<UDRE)) ) ;
      uartN(_send_fifo_char)();
      FIFOBUF_PUSH( &uartN(_tx_buf), v );
    } else {
      while( uartN(_send_nowait)(v) < 0 ) ;
    }
  }
  return 0;
}


int uartN(_send_nowait)(uint8_t v)
{
  uint8_t flags;
  IRQ_LOCK(flags);

  if( FIFOBUF_ISFULL( &uartN(_tx_buf) ) ) {
    IRQ_UNLOCK(flags);
    return -1;
  }

  FIFOBUF_PUSH( &uartN(_tx_buf), v );
  N_(UCSR,B) |= (1<<UDRIE) | (1<<TXEN);  // TXEN may have been disabled

  IRQ_UNLOCK(flags);
  return 0;
}


void uartN(_disable_tx)(void)
{
  N_(UCSR,B) |= (1<<TXCIE);
}


int uartN(_dev_recv)(FILE *fp)
{
  (void)fp; // not used
  return uartN(_recv)();
}

int uartN(_dev_send)(char c, FILE *fp)
{
  (void)fp; // not used
  return uartN(_send)(c);
}


/// Interruption handler for received data.
SIGNAL(SIG_UARTN(RECV))
{
  uint8_t v = N_(UDR,); // always read to clear RXC
  if( !FIFOBUF_ISFULL( &uartN(_rx_buf) ) ) {
    FIFOBUF_PUSH( &uartN(_rx_buf), v);
  }
}


/// Interruption handler for sent data.
SIGNAL(SIG_UARTN(DATA))
{
  uartN(_send_fifo_char)();
}


/// Interruption handler to disable TX after sending a frame.
SIGNAL(SIG_UARTN(TRANS))
{
  N_(UCSR,B) &= ~(1<<TXEN);
}



#undef UARTN
#undef uartN
#undef SIG_UARTN
#undef N_
