-- Lua script.
p=tetview:new()
p:load_mesh("E:/temp/tetgen1.6.0/tetgen1.6.0/Build/Debug/yuanzhu.1.ele")
rnd=glvCreate(0, 0, 500, 500, "TetView")
p:plot(rnd)
glvWait()
