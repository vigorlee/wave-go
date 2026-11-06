/*
 * @Author: zhangjian zhangjian@jushenzhiren.com
 * @Date: 2024-12-19 16:49:08
 * @LastEditors: zhangjian zhangjian@jushenzhiren.com
 * @LastEditTime: 2024-12-28 16:27:33
 * @FilePath: /navigo_sdk/include/udp.h
 */
#pragma once

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <functional>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>

using namespace std;
using namespace boost::asio;

namespace navigo_sdk {

template <typename T> using mp_func_async_receive = std::function<void(T &)>;

template <typename T1, typename T2>

class Udp {
public:
  Udp(const string &serverIp, const string &clientIp, const int serverPort,
      const int clientPort)
      : m_server_ip(serverIp), m_client_ip(clientIp), m_server_port(serverPort),
        m_client_port(clientPort), m_sock(m_ios), timer_(m_ios) {
  }
  ~Udp() {
    std::cout << "~udp" << std::endl;
    try {
      m_sock.shutdown(boost::asio::ip::udp::socket::shutdown_both);
    } catch (const std::exception &e) {
      std::cerr << e.what() << '\n';
    }
    m_sock.close();
    m_run_thread->join();
  }

  bool init_async() {
    mp_send_ep = make_shared<ip::udp::endpoint>(
        ip::address::from_string(m_server_ip), m_server_port);
    mp_recv_ep = make_shared<ip::udp::endpoint>(
        ip::address::from_string(m_client_ip), m_client_port);
    if (sock_open() && client_bind()) {
      recvMsg();
      m_run_thread.reset(new thread([this]() { m_ios.run(); }));
      return true;
    } else {
      printf("init error\n");
      return false;
    }
  }

  bool sendMsg(T2 *send_model) {
    memcpy(m_send_buf, send_model, sizeof(T2));
    m_sock.send_to(buffer(m_send_buf, sizeof(T2)), *mp_send_ep, 0, m_ec);
    if (m_ec) {
      std::cerr << "send: " << m_ec << '\n';
      return false;
    }
    return true;
  }

  void reg_async_receive(mp_func_async_receive<T1> func) {
    mp_reg_async_receive = func;
  }

  void delay_timer() {
    timer_.expires_after(std::chrono::seconds(5));
    timer_.async_wait([this](const boost::system::error_code &ec) {
      if (!ec) {
        time_out_ = true;
        // m_sock.close();
      }
    });
  }

  bool is_time_out() { return time_out_; }

private:
  io_service m_ios;
  string m_server_ip;
  string m_client_ip;
  int m_server_port;
  int m_client_port;
  ip::udp::socket m_sock;
  boost::system::error_code m_ec;
  shared_ptr<ip::udp::endpoint> mp_send_ep;
  shared_ptr<ip::udp::endpoint> mp_recv_ep;
  boost::asio::steady_timer timer_;
  bool time_out_ = false;
  bool start_timer = false;
  char m_send_buf[sizeof(T2)];
  char m_receive_buf[sizeof(T1)];

  mp_func_async_receive<T1> mp_reg_async_receive = nullptr;

  unique_ptr<thread> m_run_thread;

  bool sock_open() {
    m_sock.open(ip::udp::v4(), m_ec);
    if (m_ec) {
      std::cerr << "open: " << m_ec.message() << '\n';
      return false;
    }
    boost::asio::socket_base::reuse_address option(true);
    m_sock.set_option(option);
    return true;
  }
  bool client_bind() {
    m_sock.bind(*mp_recv_ep, m_ec);
    std::cout << "mp_recv_cp: " << *mp_recv_ep << std::endl;
    if (m_ec) {
      std::cerr << "bind: " << m_ec.message() << '\n';
      return false;
    }
    return true;
  }

  void recvMsg() {
    start_timer = false;
    if (start_timer) {
      delay_timer();
    }
    auto buf = buffer(m_receive_buf, sizeof(T1));
    m_sock.async_receive_from(
        buf, *mp_recv_ep, [this](const std::error_code &ec, size_t len) {
          (void)len;
          if (!ec) {
            start_timer = true;
            timer_.cancel();
            mp_reg_async_receive(*reinterpret_cast<T1 *>(m_receive_buf));
            recvMsg();

          } else {
            start_timer = false;
            cout << "error async_receive_from" << endl;
          }
        });
  }
};
} // namespace navigo_sdk