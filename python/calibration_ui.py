"""Local calibration and D18 arming page. Calibration writes never move hardware."""
import copy
import json
import os
import secrets
import time
from pathlib import Path
from target_lock import lock_config, DEFAULTS as LOCK_DEFAULTS
from auto_motion import MOTION_DEFAULTS,VISIBILITY_DEFAULTS,checked_settings
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import HTMLResponse, Response
from auto_controller import (load_auto_config, load_rear_config, infer_image_direction, sensor_pin,
                            AutoFault, near_config, NEAR_DEFAULTS, approach_config, APPROACH_DEFAULTS,
                            tracking_config, TRACKING_DEFAULTS)

PAGE = r'''<!doctype html><html lang="vi"><meta charset="utf-8"><title>Hiệu chuẩn AUTO</title>
<style>body{font:16px system-ui;background:#f1f5f9;color:#0f172a;max-width:1000px;margin:24px auto;padding:16px}section{background:white;padding:20px;border-radius:16px;margin:16px 0}button,input,select{font:inherit;padding:8px;margin:4px;border-radius:6px;border:1px solid #94a3b8}button{cursor:pointer;background:#dcfce7}img{max-width:100%;border:1px solid #ccc;cursor:crosshair}pre{white-space:pre-wrap}label{display:inline-block}#msg{color:#0f766e}</style>
<h1>AUTO khoảng cách mặt sàn — V3.10</h1><p>Chỉnh tay/bánh bằng MANUAL trên app Vercel. Dừng AUTO trước khi sửa thông số.</p>
<section><h2>Khoảng cách cm để dừng bánh</h2><p>Đưa tay về SEARCH, giữ robot đứng yên. Chọn một mốc cố định ở mép trước xe, chiếu xuống sàn làm gốc: X ngang (phải dương), Y về trước xe (dương). Đặt ít nhất 6 dấu trên sàn phẳng; dùng thước đo X,Y của từng dấu. Các dấu phải bao phủ vùng chai từ xa tới vùng dừng. Không dùng tâm hoặc đỉnh chai làm điểm sàn.</p><p>Chọn ít nhất 4 FIT bao quanh vùng quan sát và ít nhất 2 CHECK khác, nằm trong vùng FIT để kiểm chứng. Giữ camera, độ zoom, mặt sàn và tư thế SEARCH như khi đo. Khoảng dừng phải được đo khi kẹp thật có thể tới chai, không có giá trị mặc định.</p><button onclick="distanceCapture()">Chụp ảnh SEARCH để chọn dấu sàn</button><img id="distanceframe" alt="Ảnh nguyên bản để chọn các điểm sàn" onclick="distancePick(event)"><p id="distancepoint">Chưa chụp ảnh</p><label>X đo bằng thước (cm)<input id="dxcm" type="number" step="0.1"></label><label>Y đo bằng thước (cm)<input id="dycm" type="number" min="1" step="0.1"></label><label>Vai trò<select id="drole"><option value="fit">FIT</option><option value="check">CHECK độc lập</option></select></label><button onclick="distanceAdd()">Thêm điểm đang chọn</button><button onclick="distanceUndo()">Xóa điểm cuối</button><pre id="distancepoints">Chưa có điểm đo</pre><label>Khoảng dừng Y từ mốc trước xe (cm)<input id="dstop" type="number" min="3" max="200" step="0.1"></label><label>Dung sai dừng (cm)<input id="dtolerance" type="number" min="0.5" max="10" value="3" step="0.1"></label><label>Sai số kiểm chứng tối đa (cm)<input id="derror" type="number" min="0.1" max="5" value="2" step="0.1"></label><p><label><input id="dfloor" type="checkbox">Các điểm là dấu trên cùng mặt sàn; chân chai nhìn thấy và nằm trong vùng đã đo</label></p><button onclick="distanceSave()">Kiểm chứng và bật dừng theo cm</button><p id="distancestatus"></p><p>Chưa có hiệu chuẩn hợp lệ → không START AUTO theo cm. Điểm ngoài vùng đã đo, ảnh đổi tỷ lệ hoặc tay rời SEARCH → không ngoại suy khoảng cách. Mốc cũ từ giấy 17 cm chỉ là tham khảo, không tự dùng làm khoảng cách tới xe.</p></section>
<section><h2>Camera trực tiếp — các mốc kích hoạt</h2><p>Ảnh xem có mốc AUTO; ảnh gửi Modal và ảnh chụp hiệu chuẩn vẫn nguyên bản. Không vẽ box vật.</p><label><input id="showguides" type="checkbox" checked>Hiện mốc kích hoạt</label><img id="liveboxes" alt="Camera với mốc AUTO" src="/api/preview?guides=true"><ul><li>Xanh lam: hai biên căn ngang bánh và đường đáy vật để chuyển bánh → tay.</li><li>Xanh lá: vùng tâm mục tiêu của tay, không phải box nhận diện. Chỉ kẹp khi căn đúng và D19 xác nhận.</li><li>Vàng: biên vùng mờ kẹp và vùng bên dưới; xám: biên bảo vệ ảnh.</li><li>Các đường theo auto_config.json, không phải cm hay điểm gắp đã hiệu chuẩn. rw/rh cũng có thể chuyển sang tay sớm hơn đường đáy.</li><li>Nhìn sau D18: ẩn mốc phía trước để tránh nhầm. Trong AUTO dùng cấu hình đã chốt lúc START; dừng rồi lưu, START lại để áp dụng.</li></ul><pre id="lockstatus"></pre></section>
<section><h2>Chuyển động mềm và vùng càng che</h2><p>Dùng firmware V3.9 hoặc mới hơn. V3.10 đặt bước tay 50 µs. Khi bật distance, khoảng cách cm quyết định chuyển tay; mốc 54% chỉ còn dùng trong chế độ ảnh cũ khi tắt distance. Biên mờ vẫn bảo vệ tầm nhìn. Bánh không chạy khi tay hạ. Dừng AUTO trước khi lưu.</p><h3>motion</h3><textarea id="motionconfig" rows="5" style="width:95%"></textarea><h3>visibility</h3><textarea id="visibilityconfig" rows="8" style="width:95%"></textarea><br><button onclick="motionSettings()">Lưu chuyển động và vùng ảnh</button></section>
<section><h2>Khóa vật và AUTO liên tục</h2><p>enabled bật khóa vật. continuous_auto giữ AUTO qua nhiều lượt tới khi STOP. Sửa thông số bên dưới khi đã dừng AUTO. Đơn vị tọa độ là tỷ lệ ảnh, không phải cm.</p><textarea id="lockconfig" rows="9" style="width:95%;font:14px monospace"></textarea><br><button onclick="lockSettings()">Lưu khóa vật</button></section>
<section><h2>Nhận rác phía sau — D18</h2><p>D18 phát hiện → nhìn sau → chụp ảnh gửi model → xoay ngăn → mở nắp. Robot chờ D18 hết phát hiện rồi đóng nắp và trở về SEARCH. START AUTO mặc định bật chức năng này.</p><p>Bật riêng phía sau khi Arduino đã kết nối và camera có ảnh. Vật đang nằm trước D18 cũng được xử lý ngay sau khi bật.</p><button onclick="rearMode(true)">Bật nhận rác D18</button><button onclick="rearMode(false)">Tắt nhận rác D18</button><span id="rearstatus"></span><p>Trong lúc chu trình đang chạy, dùng DỪNG trên app robot để hủy.</p></section>
<p><b>SEARCH đã lấy từ trao đổi:</b> P1/S2 1300 · P10/S1 950 · P11/trụ 1250 · D2/nối 2100 · D3 mở 600 µs. S1, trụ và nối giữ nguyên khi hạ.</p>
<section><h2>1. Bánh tiến gần trước, tay hạ sau</h2><p>Tay giữ SEARCH khi bánh căn ngang và tiến FORWARD từng nhịp. Chỉ chuyển sang hạ khi vật căn ngang đủ ảnh ổn định và đạt ít nhất một mốc: đáy bbox, rộng bbox hoặc cao bbox. Đây là các mốc tương đối cần thử trên robot, không phải centimet. Khi hạ S2, bánh đứng yên; lệch ngang quá mức, ảnh mất/quá gần hoặc tới giới hạn hạ thì giữ vị trí.</p>
<label>Bật luồng mới<input id="aenabled" type="checkbox" checked></label>
<label>PWM bánh<input id="aspeed" type="number" min="1" max="255" value="20"></label>
<div id="approachfields"></div><label>Cho phép đổi tên cùng ngăn khi bbox liên tục<input id="tallow" type="checkbox" checked></label>
<label>IoU tối thiểu<input id="tiou" type="number" min="0.1" max="0.9" step="0.05" value="0.3"></label>
<label>Tâm đổi tối đa %<input id="tshift" type="number" min="1" max="20" value="8"></label>
<button onclick="approachSettings()">Lưu tiến gần và theo dõi vật</button><p>Khi distance bật, điều kiện dừng dùng cm đã hiệu chuẩn, không dùng đáy 54% hoặc rw/rh để kết luận đủ gần. Khi distance tắt, các mốc ảnh phía trên có hiệu lực như V3.9. Bộ giấy 17 cm cũ chỉ được giữ làm tham khảo.</p></section>
<section><h2>AUTO tương đối — không bắt buộc ghi mốc S2</h2><p>Điểm tham chiếu mặc định nằm giữa ảnh (50%, 50%), sai số ngang ±3%, dọc ±8%. Python nhận diện lại sau mỗi nhịp bánh và bước S2, cập nhật chiều bù theo phản hồi ảnh. Đây là tham chiếu thử ban đầu, không phải điểm gắp đã hiệu chuẩn.</p>
<label>Mục tiêu ngang %<input id="rx" type="number" value="50" min="0.1" max="99.9" step="0.1"></label>
<label>Mục tiêu dọc %<input id="ry" type="number" value="50" min="0.1" max="99.9" step="0.1"></label>
<label>Sai số ngang %<input id="rtx" type="number" value="3" min="0.1" max="24" step="0.1"></label>
<label>Sai số dọc %<input id="rty" type="number" value="8" min="0.1" max="24" step="0.1"></label>
<label>Giới hạn hạ S2 µs<input id="rlimit" type="number" value="2900" min="500" max="3500"></label>
<label>Lặp chu trình<input id="rrepeat" type="checkbox"></label><button onclick="relativeSettings()">Dùng AUTO tương đối</button>
<p>D19 phát hiện và vật đã vào vùng tham chiếu thì dừng hạ, đóng kẹp. Nếu tới giới hạn mà IR chưa thấy vật, hoặc IR kích hoạt khi vật còn lệch, AUTO dừng. D19 kiểm tra hiện diện sau nâng; D18 giữ chức năng cảm biến thùng rác. Camera USB index 0.</p></section>
<section><h2>Vật lớn / sát mép ảnh — chuyển sang chờ D19</h2><p>rw = rộng khung bao vật / rộng ảnh. Tới ngưỡng rw hoặc khung chạm vùng mép thì bánh và tay dừng, không gửi thêm ảnh căn vật. Chỉ gắp khi đã có lần căn ổn định gần đây, bánh chưa chạy lại, S2 đi không quá một bước và IR xác nhận. Chưa đủ điều kiện thì giữ xung tay, tắt D18 và chờ bạn xử lý. Camera trực tiếp vẫn chạy.</p>
<label>Bật giới hạn ảnh<input id="nenabled" type="checkbox" checked></label>
<label>Ngưỡng rw %<input id="nrw" type="number" min="20" max="95" step="1" value="60"></label>
<label>Vùng mép ảnh %<input id="nedge" type="number" min="0" max="5" step="0.1" value="1"></label>
<label>Tuổi lần căn tối đa (giây)<input id="nage" type="number" min="1" max="60" value="30"></label>
<label>Chờ IR tối đa (ms)<input id="nwait" type="number" min="300" max="10000" value="2000"></label>
<label>IR liên tục (ms)<input id="nconfirm" type="number" min="200" max="2000" value="300"></label>
<button onclick="nearSettings()">Lưu giới hạn ảnh</button><p>60% là ngưỡng thử ban đầu, không phải khoảng cách gắp đã đo. Chọn vùng mép 0% nếu chỉ muốn tính lúc khung chạm đúng biên ảnh.</p></section>
<details><summary>Tuỳ chọn: dùng các mốc đã ghi và chiều bánh cố định</summary>
<section><h2>1. Chọn điểm tương đối khi hiệu chuẩn</h2><p>Đặt vật thử tại vị trí giữa hai má kẹp khi tay hạ đến vị trí gắp thực tế. Giữ xe và vật đứng yên trong lúc ghi các mốc. Chỉnh S2, chờ tay dừng, chụp rồi nhấn gần <b>tâm vật/tâm bbox</b>; không cần nhập tọa độ chính xác.</p>
<button onclick="capture()">Chụp ảnh hiện tại</button><span id="pose"></span><br><img id="frame" alt="Ảnh camera để chọn điểm" onclick="pick(event)"><p id="point">Chưa chọn điểm</p>
<label>Sai số ngang % chiều rộng<input id="tx" type="number" value="2.5" min="0.1" max="24" step="0.1"></label><label>Sai số dọc % chiều cao<input id="ty" type="number" value="3.5" min="0.1" max="24" step="0.1"></label>
<button onclick="record()">Lưu mốc S2 và điểm tương đối</button><p>Ghi ít nhất SEARCH 1300 µs và mốc gắp cuối. Python nội suy điểm tương đối ở các xung giữa hai mốc; có thể ghi thêm 1700, 2100, 2500 µs nếu vật dịch cong trên ảnh. Đây là nội suy thử nghiệm, không phải khoảng cách cm. Mốc 2900 µs/90° còn cao 12–14 cm thì chưa phải mốc gắp trên sàn.</p></section>
<section><h2>2. Chiều bánh xe và cảm biến</h2>
<p><b>Chiều ảnh:</b> x tăng sang phải, y tăng xuống dưới. “Chiều bù” là hướng chạy xe để tâm vật trở về điểm mục tiêu, không phải hướng vật dịch.</p>
<table><tr><th>Nhịp thử bằng MANUAL</th><th>Vật dịch trên ảnh</th><th>Chiều bù khi lệch dương</th></tr><tr><td>Xe đi RIGHT</td><td>Sang trái</td><td>Vật lệch phải → RIGHT</td></tr><tr><td>Xe đi RIGHT</td><td>Sang phải</td><td>Vật lệch phải → LEFT</td></tr><tr><td>Xe đi FORWARD</td><td>Lên trên</td><td>Vật nằm dưới mục tiêu → FORWARD</td></tr><tr><td>Xe đi FORWARD</td><td>Xuống dưới</td><td>Vật nằm dưới mục tiêu → BACK</td></tr></table>
<p>Để trang tính chiều: đưa tay về SEARCH, giữ vật trên sàn. Chụp/chọn tâm → Ghi trước nhịp thử → bấm đúng hướng trên app một nhịp ngắn → chụp/chọn lại cùng vật → Lưu chiều sau nhịp thử. Trang này không tự chạy bánh.</p>
<label>Nhịp sẽ thử<select id="trial"><option value="RIGHT">RIGHT — xe đi ngang phải</option><option value="FORWARD">FORWARD — xe tiến</option></select></label><button onclick="trialBefore()">Ghi trước nhịp thử</button><button onclick="trialAfter()">Lưu chiều sau nhịp thử</button><p id="trialmsg"></p>
<label>Vật lệch phải trên ảnh → xe đi<select id="x"><option value="">Chọn sau khi thử</option><option>RIGHT</option><option>LEFT</option></select></label>
<label>Vật nằm dưới điểm mục tiêu → xe đi<select id="y"><option value="">Chọn sau khi thử</option><option>FORWARD</option><option>BACK</option></select></label>
<p>Kiểm tra lại chiều bù ở các tư thế hạ. Nếu camera bị xoay/lật hoặc hướng dịch đổi dấu, dừng để sửa hiệu chuẩn.</p>
<label>IR phát hiện gần vật<select id="contact"><option>D19</option><option>D15</option><option>D16</option></select></label>
<label>IR kiểm tra sau nâng<select id="hold"><option>D19</option><option>D15</option><option>D16</option></select></label>
<p><b>IR D19:</b> OUT nối digital 19 Mega. Cảm biến thùng rác nối D18, giữ chức năng riêng. Một IR chỉ xác nhận vật còn trong vùng phát hiện sau nâng; không đo lực kẹp. PCA9685 không đọc được cảm biến. D15/D16 chỉ dành cho hệ dùng dây cũ.</p>
<label>Mức kích hoạt<select id="level"><option>0</option><option>1</option></select></label>
<label>Xung gắp cuối S2 µs<input id="final" type="number" value="2900" min="500" max="3500"></label>
<label>Lặp chu trình<input id="repeat" type="checkbox"></label><button onclick="settings()">Dùng các mốc đã ghi</button></section></details>
<section><h2>3. Kiểm tra</h2><button onclick="refresh()">Kiểm tra cấu hình</button><button onclick="reset()">Xóa các mốc để hiệu chuẩn lại</button><p id="msg"></p><pre id="status"></pre></section>
<script>
const token='__TOKEN__';let shot=null,point=null,trial=null;
async function api(path,body){let r=await fetch(path,{method:body?'POST':'GET',headers:body?{'Content-Type':'application/json','X-Cal-Token':token}:{},body:body?JSON.stringify(body):undefined});let d=await r.json();if(!r.ok)throw Error(d.detail||'Lỗi');return d}
async function attempt(fn){try{await fn()}catch(e){document.getElementById('msg').textContent=e.message}}
async function refresh(){let d=await api('/api/config');document.getElementById('rearstatus').textContent=d.robot.auto_rear?'Đang bật':'Đang tắt';document.getElementById('status').textContent=JSON.stringify(d,null,2);document.getElementById('msg').textContent=d.readiness;for(let [id,key] of [['x','positive_error_x_move'],['y','positive_error_y_move'],['contact','contact_sensor'],['hold','hold_sensor'],['level','active_level'],['final','grasp_final_s2_us']])document.getElementById(id).value=d.calibration[key]??'';let goal=d.calibration.relative_target_norm||[.5,.5],tol=d.calibration.relative_tolerance_norm||[.08,.08];for(let [id,value] of [['rx',goal[0]],['ry',goal[1]],['rtx',tol[0]],['rty',tol[1]]])document.getElementById(id).value=(value*100).toFixed(1);document.getElementById('rlimit').value=d.calibration.grasp_final_s2_us;document.getElementById('repeat').checked=d.control.repeat;document.getElementById('rrepeat').checked=d.control.repeat;let n=d.near||{};document.getElementById('nenabled').checked=n.enabled??true;document.getElementById('nrw').value=100*(n.stop_rw??.6);document.getElementById('nedge').value=100*(n.edge_margin_norm??.01);document.getElementById('nage').value=n.max_alignment_age_s??30;document.getElementById('nwait').value=n.wait_contact_ms??2000;document.getElementById('nconfirm').value=n.contact_confirm_ms??300;fillApproach(d);document.getElementById('lockconfig').value=JSON.stringify(d.target_lock,null,2);document.getElementById('motionconfig').value=JSON.stringify(d.motion,null,2);document.getElementById('visibilityconfig').value=JSON.stringify(d.visibility,null,2);}
function capture(){attempt(async()=>{shot=await api('/api/capture');point=null;document.getElementById('frame').src=shot.image;document.getElementById('pose').textContent='S2 '+shot.s2_us+' µs · '+shot.frame_size.join(' × ');document.getElementById('point').textContent='Nhấn lên ảnh để chọn tâm vật';})}
function pick(e){if(!shot)return;let rect=e.target.getBoundingClientRect();point=[(e.clientX-rect.left)*shot.frame_size[0]/rect.width,(e.clientY-rect.top)*shot.frame_size[1]/rect.height];let norm=point.map((v,i)=>100*v/shot.frame_size[i]);document.getElementById('point').textContent='Tương đối: '+norm.map(v=>v.toFixed(1)+'%').join(', ')+' · pixel: '+point.map(v=>v.toFixed(1)).join(', ')}
function record(){attempt(async()=>{if(!shot||!point)throw Error('Chụp ảnh và chọn điểm trước');await api('/api/stage',{capture_id:shot.id,target_norm:point.map((v,i)=>v/shot.frame_size[i]),tolerance_norm:[Number(document.getElementById('tx').value)/100,Number(document.getElementById('ty').value)/100]});await refresh()})}
function trialBefore(){attempt(async()=>{if(!shot||!point)throw Error('Chụp và chọn tâm trước nhịp thử');trial={capture_id:shot.id,point:point.slice(),command:document.getElementById('trial').value};document.getElementById('trialmsg').textContent='Đã ghi trước. Chạy '+trial.command+' bằng app, dừng, chụp/chọn lại tâm rồi lưu chiều.'})}
function trialAfter(){attempt(async()=>{if(!trial||!shot||!point)throw Error('Ghi trước nhịp thử, chạy xe, chụp và chọn tâm mới');let d=await api('/api/direction',{before_id:trial.capture_id,after_id:shot.id,before_px:trial.point,after_px:point,command:trial.command});document.getElementById('trialmsg').textContent='Vật dịch '+d.delta_px.map(v=>v.toFixed(1)).join(', ')+' px. Chiều bù lệch dương: '+d.move;trial=null;await refresh()})}
function relativeSettings(){attempt(async()=>{await api('/api/relative',{target_norm:[Number(document.getElementById('rx').value)/100,Number(document.getElementById('ry').value)/100],tolerance_norm:[Number(document.getElementById('rtx').value)/100,Number(document.getElementById('rty').value)/100],grasp_final_s2_us:Number(document.getElementById('rlimit').value),repeat:document.getElementById('rrepeat').checked});await refresh()})}
function settings(){attempt(async()=>{await api('/api/settings',{mode:'calibrated',positive_error_x_move:document.getElementById('x').value||null,positive_error_y_move:document.getElementById('y').value||null,contact_sensor:document.getElementById('contact').value||null,hold_sensor:document.getElementById('hold').value||null,active_level:Number(document.getElementById('level').value),grasp_final_s2_us:Number(document.getElementById('final').value),repeat:document.getElementById('repeat').checked});await refresh()})}
function reset(){attempt(async()=>{await api('/api/reset',{});await refresh()})}refresh();
function rearMode(enabled){attempt(async()=>{await api('/api/rear',{enabled});await refresh()})}
function nearSettings(){attempt(async()=>{await api('/api/near',{enabled:document.getElementById('nenabled').checked,stop_rw:Number(document.getElementById('nrw').value)/100,edge_margin_norm:Number(document.getElementById('nedge').value)/100,max_alignment_age_s:Number(document.getElementById('nage').value),wait_contact_ms:Number(document.getElementById('nwait').value),contact_confirm_ms:Number(document.getElementById('nconfirm').value)});await refresh()})}
const approachFields=[['target_x_norm','Tâm ngang %',.5,100],['x_tolerance_norm','Sai số căn ngang %',.03,100],['stop_bottom_norm','Mốc đáy bbox %',.78,100],['stop_rw','Rộng bbox dừng tiến %',.4,100],['stop_rh','Cao bbox dừng tiến %',.7,100],['forward_ms','Nhịp tiến ms',120,1],['lateral_ms','Nhịp ngang ms',100,1],['max_forward_steps','Tối đa nhịp tiến',40,1],['max_lateral_steps','Tối đa nhịp ngang',25,1],['max_no_progress','Tối đa nhịp không phản hồi',5,1],['min_progress_norm','Phản hồi ảnh tối thiểu %',.003,100],['arm_x_tolerance_norm','Sai số ngang khi hạ tay %',.06,100]];
for(const [key,label,value,scale] of approachFields){const l=document.createElement('label');l.textContent=label;const input=document.createElement('input');input.id='ap-'+key;input.type='number';input.step='any';input.value=value*scale;l.append(input);document.getElementById('approachfields').append(l)}
function fillApproach(d){const a=d.approach||{},t=d.tracking||{};document.getElementById('aenabled').checked=a.enabled??false;document.getElementById('aspeed').value=d.control.speed;for(const [key,label,value,scale] of approachFields)document.getElementById('ap-'+key).value=(a[key]??value)*scale;document.getElementById('tallow').checked=t.allow_label_variation??false;document.getElementById('tiou').value=t.min_iou??.3;document.getElementById('tshift').value=(t.max_label_shift_norm??.08)*100}
function approachSettings(){attempt(async()=>{const a={enabled:document.getElementById('aenabled').checked};for(const [key,label,value,scale] of approachFields)a[key]=Number(document.getElementById('ap-'+key).value)/scale;await api('/api/approach',{approach:a,tracking:{allow_label_variation:document.getElementById('tallow').checked,min_iou:Number(document.getElementById('tiou').value),max_label_shift_norm:Number(document.getElementById('tshift').value)/100},speed:Number(document.getElementById('aspeed').value)});await refresh()})}
function motionSettings(){attempt(async()=>{await api('/api/motion',{motion:JSON.parse(document.getElementById('motionconfig').value),visibility:JSON.parse(document.getElementById('visibilityconfig').value)});await refresh()})}
function lockSettings(){attempt(async()=>{await api('/api/target-lock',JSON.parse(document.getElementById('lockconfig').value));await refresh()})}
let previewBusy=false;
let distanceShot=null,distancePoint=null,distancePoints=[];
function distanceList(){document.getElementById('distancepoints').textContent=distancePoints.map((p,i)=>(i+1)+' '+p.kind.toUpperCase()+' pixel '+p.pixel.map(x=>x.toFixed(1)).join(',')+' → X,Y='+p.floor_cm.join(',')+' cm').join('\n')||'Chưa có điểm đo';}
function distanceCapture(){attempt(async()=>{distanceShot=await api('/api/capture');distancePoint=null;distancePoints=[];document.getElementById('distanceframe').src=distanceShot.image;document.getElementById('distancepoint').textContent='Nhấn đúng tâm dấu sàn, nhập X,Y đo bằng thước rồi Thêm điểm';distanceList();});}
function distancePick(e){if(!distanceShot)return;let r=e.target.getBoundingClientRect();distancePoint=[(e.clientX-r.left)/r.width*distanceShot.frame_size[0],(e.clientY-r.top)/r.height*distanceShot.frame_size[1]];document.getElementById('distancepoint').textContent='Pixel '+distancePoint.map(x=>x.toFixed(1)).join(', ');}
function distanceAdd(){if(!distancePoint)return document.getElementById('distancepoint').textContent='Chọn một dấu trên ảnh trước';let a=document.getElementById('dxcm').value,b=document.getElementById('dycm').value;if(!a.trim()||!b.trim()||!Number.isFinite(Number(a))||!Number.isFinite(Number(b))||Number(b)<=0)return document.getElementById('distancepoint').textContent='Nhập X,Y đo thật bằng cm';distancePoints.push({pixel:distancePoint.slice(),floor_cm:[Number(a),Number(b)],kind:document.getElementById('drole').value});distancePoint=null;distanceList();}
function distanceUndo(){distancePoints.pop();distanceList();}
function distanceSave(){attempt(async()=>{if(!distanceShot)throw Error('Chụp ảnh SEARCH trước');if(!document.getElementById('dfloor').checked)throw Error('Xác nhận các điểm đo trên sàn');let stop=document.getElementById('dstop').value;if(!stop.trim())throw Error('Đo và nhập khoảng dừng thật');let d=await api('/api/distance',{capture_id:distanceShot.id,points:distancePoints,stop_forward_cm:Number(stop),tolerance_cm:Number(document.getElementById('dtolerance').value),max_error_cm:Number(document.getElementById('derror').value),confirm_floor:true});document.getElementById('distancestatus').textContent='Đã kiểm chứng: sai số lớn nhất '+d.max_observed_error_cm.toFixed(2)+' cm. Dừng tại Y='+d.stop_forward_cm+' cm. START lại để áp dụng.';await refresh();});}
api('/api/config').then(d=>{let c=d.distance||{};if(c.stop_forward_cm!=null)document.getElementById('dstop').value=c.stop_forward_cm;document.getElementById('dtolerance').value=c.tolerance_cm??3;document.getElementById('derror').value=c.max_error_cm??2;document.getElementById('distancestatus').textContent=c.enabled&&c.calibration?'Đã lưu hiệu chuẩn; START dùng khoảng dừng '+c.stop_forward_cm+' cm.':'Chưa bật hiệu chuẩn khoảng cách cm hợp lệ.';}).catch(()=>{});
setInterval(async()=>{if(previewBusy||document.hidden)return;previewBusy=true;try{document.getElementById('liveboxes').src='/api/preview?guides='+document.getElementById('showguides').checked+'&t='+Date.now();let d=await api('/api/boxes');document.getElementById('lockstatus').textContent=d.state+' · '+d.message}catch(e){}finally{previewBusy=false}},750);
</script></html>'''


def calibration_app(brain, vision, controller, camera_guides=None):
    from base64 import b64encode
    app = FastAPI(docs_url=None,redoc_url=None,openapi_url=None)
    from operator_ui import mount_operator
    mount_operator(app, brain, controller)
    token = secrets.token_urlsafe(24)
    captures = {}
    path = controller.path
    if camera_guides is None:
        from camera_guides import CameraGuides
        camera_guides = CameraGuides(path, controller, brain)

    from config_io import read_config
    def read(): return read_config(path)
    def write(c):
        if path.suffix == '.py':
            raise HTTPException(409,'Sửa trực tiếp robot_params.py rồi chạy lại Python; trang này chỉ xem camera')
        temp = path.with_suffix('.tmp')
        with temp.open('w',encoding='utf-8') as f:
            json.dump(c,f,ensure_ascii=False,indent=2);f.flush();os.fsync(f.fileno())
        os.replace(temp,path)

    async def mutation(request):
        if path.suffix == '.py': raise HTTPException(409,'Dùng robot_params.py để sửa tham số')
        if request.headers.get('x-cal-token') != token: raise HTTPException(403,'Sai token trang hiệu chuẩn')
        if controller.running or brain.auto_owner: raise HTTPException(409,'Dừng AUTO trước khi thay hiệu chuẩn')
        return await request.json()

    @app.get('/',response_class=HTMLResponse)
    def index(): return PAGE.replace('__TOKEN__',token)

    @app.post('/api/target-lock')
    async def lock_settings(request:Request):
        body=await mutation(request)
        try:checked=lock_config(body)
        except ValueError as exc:raise HTTPException(400,str(exc)) from exc
        c=read();c['target_lock']=checked;write(c)
        return {'saved':True,'target_lock':checked}

    @app.get('/api/boxes')
    def boxes():
        overlay=getattr(vision,'overlay',None)
        base={'state':controller.state['state'],'message':controller.state['message'],'auto_running':controller.state['auto_running']}
        if not overlay:return dict(base,objects=[],selected=None)
        age=max(0,time.monotonic()-overlay['captured'])
        return dict(base,age_s=round(age,2),stale=age>overlay['max_age'],target_id=overlay['target_id'],
                    locked_bin=overlay['locked_bin'],selected=overlay['selected'],
                    objects=[{'bbox':o.get('bbox'),'label':o.get('label'),'bin':o.get('bin')} for o in overlay['objects']])

    @app.get('/api/preview')
    def preview(guides: bool = False):
        if not vision or not vision.jpeg or time.monotonic()-vision.frame_at>2:
            raise HTTPException(409,'Camera chưa có ảnh mới')
        image = vision.jpeg
        if guides:
            try: image = camera_guides.jpeg(image)
            except Exception as exc: raise HTTPException(503,'Chưa vẽ được mốc camera') from exc
        return Response(image,media_type='image/jpeg',headers={'Cache-Control':'no-store'})

    @app.post('/api/motion')
    async def motion_settings(request:Request):
        body=await mutation(request)
        if not isinstance(body,dict) or set(body)!={'motion','visibility'}:raise HTTPException(400,'Sai trường chuyển động')
        try:
            m=checked_settings(body['motion'],MOTION_DEFAULTS)
            v=checked_settings(body['visibility'],VISIBILITY_DEFAULTS)
            if v['enabled'] and (not m['enabled'] or v['min_step_us']>m['arm_step_us']):raise ValueError('Sai phối hợp tay/bánh')
        except ValueError as exc:raise HTTPException(400,str(exc)) from exc
        c=read()
        if v['enabled'] and (not c.get('approach',{}).get('enabled') or c['calibration']['mode']!='relative'):
            raise HTTPException(400,'Giữ ảnh cần approach relative')
        c['motion']=m;c['visibility']=v;write(c)
        return {'saved':True,'motion':m,'visibility':v}

    @app.post('/api/distance')
    async def distance_settings(request:Request):
        body = await mutation(request)
        from floor_distance import fit_calibration, distance_config, DEFAULTS
        from camera_publisher import CAMERA_INDEX
        if not isinstance(body,dict) or body.get('confirm_floor') is not True:
            raise HTTPException(400,'Cần xác nhận các điểm đo thật trên mặt sàn')
        shot = captures.get(body.get('capture_id'))
        if not shot or time.monotonic()-shot['at'] > 300:
            raise HTTPException(409,'Ảnh đã quá 5 phút; chụp và chọn điểm lại')
        if not vision or not getattr(vision,'prefer_local',False):
            raise HTTPException(409,'Hiệu chuẩn cm cần camera USB tại máy Python, LIVEKIT_PUBLISHER=1')
        s = brain.snapshot(); search={j:brain.config['poses']['SEARCH'][j] for j in ('p1','p10','p11','d2')}
        if not s['fresh'] or s['busy'] or s['drive']['active'] or any(s['hardware']['joints'].get(j)!=p for j,p in search.items()):
            raise HTTPException(409,'Đưa về SEARCH và giữ bánh/tay đứng yên trước khi lưu')
        if shot['s2_us']!=search['p1'] or shot['frozen']!={j:search[j] for j in ('p10','p11','d2')}:
            raise HTTPException(409,'Ảnh phải chụp đúng tư thế SEARCH')
        try:
            from floor_distance import finite
            error=finite(body.get('max_error_cm'),.1,5,'max_error_cm')
            cal=fit_calibration(body.get('points'),shot['frame_size'],search,CAMERA_INDEX,error)
            cfg=distance_config(dict(DEFAULTS,enabled=True,stop_forward_cm=body.get('stop_forward_cm'),
                                tolerance_cm=body.get('tolerance_cm'),max_error_cm=error,calibration=cal),brain.config,CAMERA_INDEX)
        except (ValueError,KeyError,TypeError) as exc:raise HTTPException(400,str(exc)) from exc
        c=read();c['distance']=cfg;write(c)
        return {'saved':True,'max_observed_error_cm':cal['max_observed_error_cm'],'stop_forward_cm':cfg['stop_forward_cm']}

    @app.get('/api/config')
    def config():
        c = read()
        try:
            checked = load_auto_config(path,brain.config)
            ready = 'AUTO tương đối sẵn cấu hình; không cần ghi từng mốc S2. Điểm tham chiếu là ước lượng, thử một lượt trước khi lặp.' if checked['calibration']['mode']=='relative' else 'Cấu hình các mốc đủ trường. Thử một lượt trước khi lặp.'
        except Exception as exc: ready = str(exc)
        return {'readiness':ready,'calibration':c['calibration'],'control':c['control'],
                'rear':c.get('rear',{}),'near':c.get('near',NEAR_DEFAULTS),'auto':copy.deepcopy(controller.state),
                'target_lock':c.get('target_lock',LOCK_DEFAULTS),'approach':c.get('approach',APPROACH_DEFAULTS),'tracking':c.get('tracking',TRACKING_DEFAULTS),
                'motion':c.get('motion',MOTION_DEFAULTS),'visibility':c.get('visibility',VISIBILITY_DEFAULTS),'distance':c.get('distance',{}),'measurements':c['measurements_reference_only'],'robot':brain.snapshot()}

    @app.post('/api/near')
    async def near_settings(request:Request):
        body = await mutation(request)
        if not isinstance(body,dict) or set(body) != set(NEAR_DEFAULTS):
            raise HTTPException(400,'Cần đủ thông số giới hạn ảnh')
        try: checked = near_config(body)
        except AutoFault as exc: raise HTTPException(400,str(exc)) from exc
        c = read()
        try: approach_config(c.get('approach',{}),checked)
        except AutoFault as exc:raise HTTPException(400,str(exc)) from exc
        c['near'] = checked;write(c)
        return {'saved':True,'near':checked}

    @app.post('/api/approach')
    async def approach_settings(request:Request):
        body=await mutation(request);c=read()
        if not isinstance(body,dict) or set(body)!={'approach','tracking','speed'}:raise HTTPException(400,'Sai trường tiến gần')
        if type(body['speed']) is not int or not 1<=body['speed']<=255:raise HTTPException(400,'PWM phải trong 1–255')
        try:
            approach=approach_config(body['approach'],near_config(c.get('near',{})))
            tracking=tracking_config(body['tracking'])
        except AutoFault as exc:raise HTTPException(400,str(exc)) from exc
        if approach['enabled'] and c['calibration'].get('mode')!='relative':raise HTTPException(400,'Chọn AUTO tương đối trước khi bật tiến gần')
        if not approach['enabled']:c.setdefault('visibility',{})['enabled']=False
        c['approach']=approach;c['tracking']=tracking;c['control']['speed']=body['speed'];write(c)
        return {'saved':True}

    @app.post('/api/rear')
    async def rear(request:Request):
        import asyncio
        body = await mutation(request)
        if not isinstance(body,dict) or set(body) != {'enabled'} or type(body['enabled']) is not bool:
            raise HTTPException(400,'enabled phải là boolean')
        if body['enabled']:
            if controller.emergency_latched:
                raise HTTPException(409,'Sau dừng khẩn cấp, cho phép điều khiển trong MANUAL trước')
            if not vision or not vision.jpeg or time.monotonic()-vision.frame_at > 2:
                raise HTTPException(409,'Camera chưa có ảnh mới; kiểm tra camera 1 trước khi bật D18')
            try: load_rear_config(path)
            except ValueError as exc: raise HTTPException(400,str(exc)) from exc
        try:
            await asyncio.to_thread(brain.command,'AUTO REAR ON' if body['enabled'] else 'AUTO REAR OFF',brain.epoch)
        except (ValueError,ConnectionError) as exc: raise HTTPException(409,str(exc)) from exc
        return {'enabled':brain.auto_rear}

    @app.get('/api/capture')
    def capture():
        s = brain.snapshot()
        if controller.running or not s['fresh'] or s['busy'] or s['drive']['active']:
            raise HTTPException(409,'Kết nối Arduino và chờ tay/bánh dừng trong MANUAL')
        if not vision or not vision.jpeg or not vision.frame_size or time.monotonic()-vision.frame_at > 2:
            raise HTTPException(409,'Camera chưa có ảnh mới')
        if vision.frame_at <= brain.motion_finished_at+.5:
            raise HTTPException(409,'Chờ thêm 0,5 giây và chụp lại sau khi tay dừng')
        joints = s['hardware']['joints']
        if any(type(joints.get(j)) is not int for j in ('p1','p10','p11','d2')): raise HTTPException(409,'Thiếu xung tay')
        ident = secrets.token_urlsafe(12)
        expired = [key for key,val in captures.items() if time.monotonic()-val['at'] > 300]
        for key in expired: captures.pop(key)
        while len(captures) >= 8: captures.pop(next(iter(captures)))
        captures[ident] = {'s2_us':joints['p1'],'frame_size':vision.frame_size[:],
            'frozen':{j:joints[j] for j in ('p10','p11','d2')},'at':time.monotonic(),'frame_at':vision.frame_at}
        return {'id':ident,'s2_us':joints['p1'],'frame_size':vision.frame_size,
                'image':'data:image/jpeg;base64,'+b64encode(vision.jpeg).decode()}

    @app.post('/api/stage')
    async def stage(request:Request):
        body = await mutation(request)
        shot = captures.get(body.get('capture_id'))
        if not shot or time.monotonic()-shot['at'] > 300: raise HTTPException(409,'Ảnh đã hết hạn; chụp lại')
        c = read();cal = c['calibration']
        if cal['stages'] and (cal['frozen_joints_us'] != shot['frozen'] or cal['frame_size'] != shot['frame_size']):
            raise HTTPException(409,'S1/trụ/nối hoặc kích thước ảnh đã đổi; xóa mốc để hiệu chuẩn lại')
        normalized = 'target_norm' in body or 'tolerance_norm' in body
        target,tolerance = (body.get('target_norm'),body.get('tolerance_norm')) if normalized else (body.get('target_px'),body.get('tolerance_px'))
        import math
        bounds = ([1,1],[.25,.25]) if normalized else (shot['frame_size'],[v/4 for v in shot['frame_size']])
        for values,high in [(target,bounds[0]),(tolerance,bounds[1])]:
            if not isinstance(values,list) or len(values)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) or not 0<v<limit for v,limit in zip(values,high)):
                raise HTTPException(400,'Sai điểm ảnh hoặc sai số')
        if not cal['stages'] and shot['s2_us'] != brain.config['poses']['SEARCH']['p1']:
            raise HTTPException(409,'Ghi mốc SEARCH trước; lưu lại tư thế SEARCH trên app nếu đã thay đổi')
        expected = {j:brain.config['poses']['SEARCH'][j] for j in ('p10','p11','d2')}
        if shot['frozen'] != expected: raise HTTPException(409,'S1/trụ/nối phải giữ đúng xung SEARCH đã lưu')
        cal['frame_size'] = shot['frame_size'];cal['frozen_joints_us'] = shot['frozen']
        cal['stages'] = [s for s in cal['stages'] if s['s2_us'] != shot['s2_us']]
        if not normalized:
            target = [v/size for v,size in zip(target,shot['frame_size'])]
            tolerance = [v/size for v,size in zip(tolerance,shot['frame_size'])]
        cal['stages'].append({'s2_us':shot['s2_us'],'target_norm':target,'tolerance_norm':tolerance})
        c['version'] = max(c.get('version',1),2)
        sign = 1 if cal['grasp_final_s2_us'] >= brain.config['poses']['SEARCH']['p1'] else -1
        cal['stages'].sort(key=lambda s:sign*s['s2_us'])
        write(c)
        return {'saved':True}

    @app.post('/api/direction')
    async def direction(request:Request):
        body = await mutation(request)
        before,after = captures.get(body.get('before_id')),captures.get(body.get('after_id'))
        if not before or not after or any(time.monotonic()-s['at'] > 300 for s in (before,after)):
            raise HTTPException(409,'Ảnh nhịp thử đã hết hạn; chụp lại')
        if after['frame_at'] <= before['frame_at']: raise HTTPException(409,'Cần ảnh mới sau nhịp thử')
        if any(before[k] != after[k] for k in ('s2_us','frame_size','frozen')):
            raise HTTPException(409,'Giữ nguyên tay và kích thước ảnh trong nhịp thử bánh')
        search = brain.config['poses']['SEARCH']
        if before['s2_us'] != search['p1'] or before['frozen'] != {j:search[j] for j in ('p10','p11','d2')}:
            raise HTTPException(409,'Thử chiều bánh ở đúng tư thế SEARCH')
        try: move,delta = infer_image_direction(body.get('before_px'),body.get('after_px'),body.get('command'),before['frame_size'])
        except AutoFault as exc: raise HTTPException(400,str(exc))
        c = read();cal=c['calibration']
        key = 'positive_error_x_move' if body['command']=='RIGHT' else 'positive_error_y_move'
        cal[key]=move
        cal.setdefault('direction_trials',{})[key] = {'s2_us':before['s2_us'],'command':body['command'],
            'delta_norm':[v/size for v,size in zip(delta,before['frame_size'])],
            'frozen_joints_us':before['frozen']}
        write(c)
        return {'saved':True,'move':move,'delta_px':delta}

    @app.post('/api/settings')
    async def settings(request:Request):
        b = await mutation(request);c = read()
        allowed = {'positive_error_x_move','positive_error_y_move','contact_sensor','hold_sensor','active_level','grasp_final_s2_us','repeat'}
        if set(b) not in (allowed,allowed|{'mode'}): raise HTTPException(400,'Sai trường cấu hình')
        if b.get('mode',c['calibration'].get('mode','calibrated')) not in ('relative','calibrated'):
            raise HTTPException(400,'Sai chế độ căn ảnh')
        if b['positive_error_x_move'] not in (None,'LEFT','RIGHT') or b['positive_error_y_move'] not in (None,'FORWARD','BACK'):
            raise HTTPException(400,'Sai chiều bánh')
        if any(b[k] is not None and sensor_pin(b[k]) is None for k in ('contact_sensor','hold_sensor')): raise HTTPException(400,'Sai cảm biến')
        if type(b['active_level']) is not int or b['active_level'] not in (0,1) or type(b['repeat']) is not bool:
            raise HTTPException(400,'Sai mức IR/lặp')
        if type(b['grasp_final_s2_us']) is not int or not brain.config['limits']['p1'][0] <= b['grasp_final_s2_us'] <= brain.config['limits']['p1'][1]:
            raise HTTPException(400,'Sai xung S2 cuối')
        for k in allowed-{'repeat'}: c['calibration'][k] = b[k]
        if 'mode' in b:
            c['calibration']['mode']=b['mode']
            if b['mode']=='calibrated':
                c.setdefault('approach',{})['enabled']=False
                c.setdefault('visibility',{})['enabled']=False
        c['control']['repeat'] = b['repeat'];write(c)
        return {'saved':True}

    @app.post('/api/relative')
    async def relative_settings(request:Request):
        from auto_controller import relative_stage
        b=await mutation(request);c=read()
        if set(b)!={'target_norm','tolerance_norm','grasp_final_s2_us','repeat'}:
            raise HTTPException(400,'Sai trường AUTO tương đối')
        try: relative_stage(b,c['calibration']['frame_size'])
        except AutoFault as exc: raise HTTPException(400,str(exc))
        final=b['grasp_final_s2_us']
        search=brain.config['poses']['SEARCH']
        if type(final) is not int or not search['p1'] < final <= brain.config['limits']['p1'][1]:
            raise HTTPException(400,'Giới hạn hạ phải lớn hơn SEARCH và không vượt giới hạn S2')
        if type(b['repeat']) is not bool: raise HTTPException(400,'Sai giá trị lặp')
        c['version']=3
        cal=c['calibration'];cal.update(mode='relative',relative_target_norm=b['target_norm'],
            relative_tolerance_norm=b['tolerance_norm'],grasp_final_s2_us=final,
            frozen_joints_us={j:search[j] for j in ('p10','p11','d2')})
        c['control']['repeat']=b['repeat'];write(c)
        return {'saved':True}

    @app.post('/api/reset')
    async def reset(request:Request):
        await mutation(request);c=read();c['calibration']['stages']=[]
        c['calibration']['frozen_joints_us']={j:brain.config['poses']['SEARCH'][j] for j in ('p10','p11','d2')}
        write(c)
        return {'saved':True}
    return app
