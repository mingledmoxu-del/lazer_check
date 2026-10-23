# v3.9.4 六套预载与业务逐点采集API

H743主工程：烧录 `LightCheck.hex`（起始地址0x08000000）；解压 `upper-computer-sdk-source.zip`。Python业务软件从 `app` 导入 `MotorController`；调试GUI运行 `app/start_simple.bat`。

板端RAM库扩为6套，每套1～16个双轴绝对点；6/8/10个点使用同一API。SDK/GUI及协议解析同步支持编号1～6，库5、6需要本版固件。起终原点若写入路线，也计入每库16点。业务软件本地可以保存更多路线，选中最多6套后上传。

业务层在自己的界面选点，再调用 `controller.prepare_point_scan(points, name=..., route_id=..., speed_rpm=3, tolerance_deg=.2)`，仅上传，不运动。`scan.move_next_and_wait()`只走一处并等待实际停稳；业务层自己采集激光、做判断，再 `scan.confirm_acquisition(index, acquisition_id, passed=True)` 开放下一点。确认不自动运动，必须再次显式调用下一点。SDK提供发送/等待分离、取消/停止和本地状态查询。同一设备上另一套PointScan不能在当前流程采集时移动电机。

接入同事或AI先读源码包 `docs/业务上位机_预载与激光逐点采集API.md`，示例 `app/examples/point_scan_business.py`。文档包含网络连接、六套不同点数路线、返回字段、采集确认、重复/迟到回调、故障/重连及原始协议入口。

速度仍是真实输出轴rpm，默认0.1；沿用v3.9.3参数块读取、日志批处理和±0.2°实际双零完成条件。本版未改定位控制或停稳参数，不新增每点启动前独立状态查询。

230项Python、15组C主机测试、H743构建、Tk六库切换检查通过。未烧录或执行实机运动/激光采集。固件与业务SDK需配套更新；断电/复位后RAM库失效，重新建立/核对参考并上传。
