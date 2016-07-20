// Copyright (c) 2016 Dominik Zeromski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

class GameServer {
public:
  GameServer();

  int ListenAndServe(const char *ip, int port);

private:
  Frame &Next(int n);

  void ApplyForce(const uv_buf_t &request, const Addr &addr);

  void OnReceive(uv_buf_t request, Addr addr);

  void NewPlayer(Addr addr);

  void OnTick();

  void OnDebugReceive(uv_buf_t request, Addr addr);

  void OnDebugTick();

  char *Alloc(int size);

  uint64_t seq_;
  Loop loop_;
  UDPServer server_;
  Timer timer_;
  std::set<Addr> clients_;
  std::map<Addr, Cube *> players_;

  QFrame qframe_;
  Frame frame_;

  UDPServer debug_server_;
  std::set<Addr> debug_clients_;

  World world_;
};