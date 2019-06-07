/*
 * This file contains TCP functions, called by user.
 * All of them must acquire/release socket lock or ip->lock,
 * as appropriate. (But never both!)
 */
#include <runtime/lib.h>
#include <kernel/uos.h>
#include <buf/buf.h>
#include <mem/mem.h>
#include <net/ip.h>
#include <net/tcp.h>
#include <timer/timer.h>

/*
 * A nastly hack featuring 'goto' statements that allocates a
 * new TCP local port.
 */
static unsigned short
tcp_new_port (ip_t *ip)
{
	tcp_socket_t *s;
again:
	++ip->tcp_port;
	if (ip->tcp_port > TCP_LOCAL_PORT_RANGE_END) {
		ip->tcp_port = TCP_LOCAL_PORT_RANGE_START;
	}
	for (s = ip->tcp_sockets; s != 0; s = s->next) {
		if (s->local_port == ip->tcp_port) {
			goto again;
		}
	}
	for (s = ip->tcp_closing_sockets; s != 0; s = s->next) {
		if (s->local_port == ip->tcp_port) {
			goto again;
		}
	}
	for (s = ip->tcp_listen_sockets; s != 0; s = s->next) {
		if (s->local_port == ip->tcp_port) {
			goto again;
		}
	}
	return ip->tcp_port;
}

/*
 * Connect to another host. Wait until connection established.
 * Return 1 on success, 0 on error.
 */
tcp_socket_t *
tcp_connect (ip_t *ip, unsigned char *ipaddr, unsigned short port)
{
	tcp_socket_t *s;
	unsigned long optdata;

	tcp_debug ("tcp_connect to port %u\n", port);
	if (ipaddr == 0)
		return 0;
	mutex_lock (&ip->lock);

	s = tcp_alloc (ip);
	memcpy (s->remote_ip, ipaddr, 4);
	s->remote_port = port;
	if (s->local_port == 0) {
		s->local_port = tcp_new_port (ip);
	}
	s->lastack = s->snd_nxt - 1;
	s->snd_lbb = s->snd_nxt - 1;
	s->snd_wnd = TCP_WND;
	s->ssthresh = s->mss * 10;
	s->state = SYN_SENT;

	/* Build an MSS option */
	optdata = HTONL (((unsigned long)2 << 24) |
		((unsigned long)4 << 16) |
		(((unsigned long)s->mss / 256) << 8) |
		(s->mss & 255));

	if (! tcp_enqueue (s, 0, 0, TCP_SYN, (unsigned char*) &optdata, 4)) {
		mem_free (s);
		mutex_unlock (&ip->lock);
		return 0;
	}
	tcp_list_add (&ip->tcp_sockets, s);
	tcp_output (s);
	mutex_unlock (&ip->lock);

	mutex_lock (&s->lock);
	for (;;) {
		mutex_wait (&s->lock);
		if (s->state == ESTABLISHED) {
			mutex_unlock (&s->lock);
			return s;
		}
		if (s->state == CLOSED) {
			mutex_unlock (&s->lock);
			mem_free (s);
			return 0;
		}
	}
}

/*
 * Send len>0 bytes.
 * Return a number ob transmitted bytes, or -1 on error.
 */
int
tcp_write (tcp_socket_t *s, const void *arg, unsigned short len)
{
	tcp_debug ("tcp_write(s=%p, arg=%p, len=%u)\n",
		(void*) s, arg, len);
	mutex_lock (&s->lock);

	if (s->state != SYN_SENT && s->state != SYN_RCVD &&
	    s->state != ESTABLISHED /*&& s->state != CLOSE_WAIT*/) {
		mutex_unlock (&s->lock);
		tcp_debug ("tcp_write() called in invalid state\n");
		return -1;
	}
	if (len == 0) {
		mutex_unlock (&s->lock);
		return -1;
	}
	mutex_group_t *g = 0;
	ARRAY (group, sizeof(mutex_group_t) + 2 * sizeof(mutex_slot_t));

	while (tcp_enqueue (s, (void*) arg, len, 0, 0, 0) == 0) {
		/* Не удалось поставить пакет в очередь - мало памяти. */
		if (! g) {
			memset (group, 0, sizeof(group));
			g = mutex_group_init (group, sizeof(group));
			mutex_group_add (g, &s->lock);
			mutex_group_add (g, &s->ip->timer->decisec);
			mutex_group_listen (g);
		}
		/* Каждые 100 мсек делаем повторную попытку. */
		mutex_unlock (&s->lock);
		mutex_group_wait (g, 0, 0);
		mutex_lock (&s->lock);

		/* Проверим, не закрылось ли соединение. */
		if (s->state != SYN_SENT && s->state != SYN_RCVD &&
		    s->state != ESTABLISHED /*&& s->state != CLOSE_WAIT*/) {
			mutex_unlock (&s->lock);
			if (g)
				mutex_group_unlisten (g);
			return -1;
		}
	}
	mutex_unlock (&s->lock);
	if (g)
		mutex_group_unlisten (g);

	mutex_lock (&s->ip->lock);
	tcp_output (s);
	mutex_unlock (&s->ip->lock);
	return len;
}

/*
 * Receive len>0 bytes. Return <0 on error.
 * When nonblock flag is zero, blocks until data get available (never returns 0).
 * When nonblock flag is nonzero, returns 0 if no data is available.
 */
int
tcp_read_poll (tcp_socket_t *s, void *arg, unsigned short len, int nonblock)
{
	buf_t *p, *q;
	char *buf;
	int n;

	tcp_debug ("tcp_read(s=%p, arg=%p, len=%u)\n",
		(void*) s, arg, len);
	if (len == 0) {
		return -1;
	}
	mutex_lock (&s->lock);
	while (tcp_queue_is_empty (s)) {
		if (s->state != SYN_SENT && s->state != SYN_RCVD &&
		    s->state != ESTABLISHED) {
			mutex_unlock (&s->lock);
			tcp_debug ("tcp_read() called in invalid state\n");
			return -1;
		}
		if (nonblock) {
			mutex_unlock (&s->lock);
			return 0;
		}
		mutex_wait (&s->lock);
	}
	p = tcp_queue_get (s);

	tcp_debug ("tcp_read: received %u bytes, wnd %u (%u).\n",
	       p->tot_len, s->rcv_wnd, TCP_WND - s->rcv_wnd);
	mutex_unlock (&s->lock);

	mutex_lock (&s->ip->lock);
	if (! (s->flags & TF_ACK_DELAY) && ! (s->flags & TF_ACK_NOW)) {
		tcp_ack (s);
	}
	mutex_unlock (&s->ip->lock);

	/* Copy all chunks. */
	buf = arg;
	n = 0;
	for (q=p; q!=0 && n<len; q=q->next) {
		int bytes;
		if (q->len == 0)
			continue;

		bytes = (q->len < len-n) ? q->len : len-n;
		memcpy (buf, q->payload, bytes);
		n += bytes;
		buf += bytes;
	}
	buf_free (p);
	return n;
}

/*
 * Blocking receive.
 * Return a number of received bytes >0.
 * Return <0 on error.
 */
int
tcp_read (tcp_socket_t *s, void *arg, unsigned short len)
{
	return tcp_read_poll (s, arg, len, 0);
}

/*
 * Set the state of the connection to be LISTEN, which means that it
 * is able to accept incoming connections. The protocol control block
 * is reallocated in order to consume less memory. Setting the
 * connection to LISTEN is an irreversible process.
 */
tcp_socket_t *tcp_listen (ip_t *ip, unsigned char *ipaddr,
	unsigned short port)
{
	tcp_socket_t *s, *cs;

	if (! ipaddr) {
		ipaddr = (unsigned char*) "\0\0\0\0";
	}
	s = mem_alloc (ip->pool, sizeof (tcp_socket_t));
	if (s == 0) {
		return 0;
	}
	s->ip = ip;

	/* Bind the connection to a local portnumber and IP address. */
	mutex_lock (&ip->lock);
	if (port == 0) {
		port = tcp_new_port (ip);
	}
	tcp_debug ("tcp_listen: port %u\n", port);

	/* Check if the address already is in use. */
	for (cs = ip->tcp_listen_sockets; cs != 0; cs = cs->next) {
		if (cs->local_port == port) {
			if (memcmp (cs->local_ip, &IP_ZERO_ADDR, 4) == 0 ||
			    memcmp (ipaddr, &IP_ZERO_ADDR, 4) == 0 ||
			    memcmp (cs->local_ip, ipaddr, 4) == 0) {
				mutex_unlock (&ip->lock);
				mem_free (s);
				return 0;
			}
		}
	}
	for (cs = ip->tcp_sockets; cs != 0; cs = cs->next) {
		if (cs->local_port == port) {
			if (memcmp (cs->local_ip, &IP_ZERO_ADDR, 4) == 0 ||
			    memcmp (ipaddr, &IP_ZERO_ADDR, 4) == 0 ||
			    memcmp (cs->local_ip, ipaddr, 4) == 0) {
				mutex_unlock (&ip->lock);
				mem_free (s);
				return 0;
			}
		}
	}

	if (memcmp (ipaddr, &IP_ZERO_ADDR, 4) != 0) {
		memcpy (s->local_ip, ipaddr, 4);
	}
	s->local_port = port;
	s->state = LISTEN;

	tcp_list_add (&ip->tcp_listen_sockets, s);
	mutex_unlock (&ip->lock);
	return s;
}

tcp_socket_t *
tcp_accept (tcp_socket_t *s)
{
	tcp_socket_t *ns;
	buf_t *p;
	ip_hdr_t *iph;
	tcp_hdr_t *h;
	unsigned long optdata;
again:
	mutex_lock (&s->lock);
	for (;;) {
		if (s->state != LISTEN) {
			mutex_unlock (&s->lock);
			tcp_debug ("tcp_accept: called in invalid state\n");
			return 0;
		}
		if (! tcp_queue_is_empty (s)) {
			p = tcp_queue_get (s);
			break;
		}
		mutex_wait (&s->lock);
	}
	mutex_unlock (&s->lock);

	/* Create a new PCB, and respond with a SYN|ACK.
	 * If a new PCB could not be created (probably due to lack of memory),
	 * we don't do anything, but rely on the sender will retransmit
	 * the SYN at a time when we have more memory available. */
	mutex_lock (&s->ip->lock);
	ns = tcp_alloc (s->ip);
	if (ns == 0) {
		tcp_debug ("tcp_accept: could not allocate PCB\n");
		++s->ip->tcp_in_discards;
		mutex_unlock (&s->ip->lock);
		buf_free (p);
		goto again;
	}
	h = (tcp_hdr_t*) p->payload;
	iph = ((ip_hdr_t*) p->payload) - 1;

	/* Set up the new PCB. */
	memcpy (ns->local_ip, iph->dest, 4);
	ns->local_port = s->local_port;
	memcpy (ns->remote_ip, iph->src, 4);
	ns->remote_port = h->src;
	ns->state = SYN_RCVD;
	ns->rcv_nxt = s->ip->tcp_input_seqno + 1;
	ns->snd_wnd = h->wnd;
	ns->ssthresh = ns->snd_wnd;
	ns->snd_wl1 = s->ip->tcp_input_seqno;
	ns->ip = s->ip;

	/* Register the new PCB so that we can begin receiving
	 * segments for it. */
	tcp_list_add (&s->ip->tcp_sockets, ns);

	/* Parse any options in the SYN. */
	tcp_parseopt (ns, h);

	/* Build an MSS option. */
	optdata = HTONL (((unsigned long)2 << 24) | ((unsigned long)4 << 16) |
		(((unsigned long)ns->mss / 256) << 8) | (ns->mss & 255));
	buf_free (p);

	/* Send a SYN|ACK together with the MSS option. */
	tcp_enqueue (ns, 0, 0, TCP_SYN | TCP_ACK, (unsigned char*) &optdata, 4);
	tcp_output (ns);
	mutex_unlock (&s->ip->lock);
	return ns;
}

/*
 * Closes the connection held by the PCB.
 * Return 1 on success, 0 on error.
 */
int
tcp_close (tcp_socket_t *s)
{
	mutex_lock (&s->lock);

	tcp_debug ("tcp_close: state=%S\n",
		tcp_state_name (s->state));

	switch (s->state) {
	default: /* Has already been closed. */
		tcp_queue_free (s);
		mutex_unlock (&s->lock);
		return 1;
	case LISTEN:
	case SYN_SENT:
		tcp_queue_free (s);
		mutex_unlock (&s->lock);
		mutex_lock (&s->ip->lock);
		tcp_socket_remove (s->state == LISTEN ?
			&s->ip->tcp_listen_sockets : &s->ip->tcp_sockets, s);
		mutex_unlock (&s->ip->lock);
		return 1;
	case SYN_RCVD:
	case ESTABLISHED:
	case CLOSE_WAIT:
		break;
	}
	for (;;) {
		if (tcp_enqueue (s, 0, 0, TCP_FIN, 0, 0))
			break;
		mutex_wait (&s->lock);
	}
	s->state = (s->state == CLOSE_WAIT) ? LAST_ACK : FIN_WAIT_1;
	mutex_unlock (&s->lock);

	mutex_lock (&s->ip->lock);
	if (s->unsent || (s->flags & TF_ACK_NOW))
		tcp_output (s);
	mutex_unlock (&s->ip->lock);

	mutex_lock (&s->lock);
	while (s->state != CLOSED) {
		if (s->state == TIME_WAIT && ! s->unsent) {
			tcp_queue_free (s);
			mutex_unlock (&s->lock);
			mutex_lock (&s->ip->lock);
			tcp_socket_remove (&s->ip->tcp_closing_sockets, s);
			mutex_unlock (&s->ip->lock);
			return 1;
		}
		mutex_wait (&s->lock);
	}
	mutex_unlock (&s->lock);
	return 1;
}

/*
 * Aborts a connection by sending a RST to the remote host and deletes
 * the local protocol control block. This is done when a connection is
 * killed because of shortage of memory.
 */
void
tcp_abort (tcp_socket_t *s)
{
	ip_t *ip = s->ip;
	unsigned long seqno, ackno;
	unsigned short remote_port, local_port;
	unsigned char remote_ip[4], local_ip[4];

	mutex_lock (&s->lock);

	/* Figure out on which TCP PCB list we are, and remove us. If we
	 * are in an active state, send an RST to the remote end. */
	if (s->state == TIME_WAIT) {
		tcp_queue_free (s);
		mutex_unlock (&s->lock);
		mutex_lock (&ip->lock);
		tcp_socket_remove (&ip->tcp_closing_sockets, s);
		mutex_unlock (&ip->lock);
		return;
	}
	seqno = s->snd_nxt;
	ackno = s->rcv_nxt;
	memcpy (local_ip, s->local_ip, 4);
	memcpy (remote_ip, s->remote_ip, 4);
	local_port = s->local_port;
	remote_port = s->remote_port;
	tcp_queue_free (s);
	mutex_unlock (&s->lock);

	mutex_lock (&ip->lock);
	tcp_socket_remove (&ip->tcp_sockets, s);
	if (s->unacked != 0) {
		tcp_segments_free (s->unacked);
	}
	if (s->unsent != 0) {
		tcp_segments_free (s->unsent);
	}

	tcp_debug ("tcp_abort: sending RST\n");
	tcp_rst (ip, seqno, ackno, local_ip, remote_ip,
		local_port, remote_port);
	mutex_unlock (&ip->lock);
}

/*
 * Return the period of socket inactivity, in seconds.
 */
unsigned long
tcp_inactivity (tcp_socket_t *s)
{
	unsigned long sec;

	mutex_lock (&s->ip->lock);
	sec = (s->ip->tcp_ticks - s->tmr) >> 1;
	mutex_unlock (&s->ip->lock);
	return sec;
}
