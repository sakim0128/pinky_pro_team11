'use strict';
const labels = ['대기','주행 중','통과 허가 대기','횡단보도 정지','횡단보도 통과','장애물 대기','차선 미검출','도착','비상정지','관제 통신 두절'];
const qualityLabels = ['양쪽 차선','한쪽 추정','분기 구간','이전 차선','차선 미검출'];
const $ = id => document.getElementById(id);
const number = (v, unit, digits=2) => typeof v === 'number' && Number.isFinite(v) ? `${v.toFixed(digits)}${unit}` : '—';
const age = sample => sample.status === 'missing' ? '미수신' : `${sample.status === 'live' ? '수신 중' : '오래된 데이터'} · ${sample.age_seconds.toFixed(1)}초 전`;
let robotNames = '';
let mapMetadata = null;
let controlsEnabled = false;
async function control(payload) {
  if (!controlsEnabled) return;
  const response = await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
  if (!response.ok) throw new Error('Control rejected');
  $('control-status').textContent=`${payload.action} 요청 전송됨`;
}
function setControlEnabled(value) {
  const changed=controlsEnabled !== value;
  controlsEnabled=value;
  ['start','pause','resume','reset'].forEach(id => { $(id).disabled=!value; });
  if (changed) $('control-status').textContent=value ? '제어 활성 · 요청 대기' : '제어 비활성';
  $('control-mode').textContent=value ? '제어 활성 · ROS 명령 전송 가능' : '조회 전용 · 제어 비활성';
  document.querySelectorAll('.linear-limit,.angular-limit,.apply').forEach(element => { element.disabled=!value; });
}
function setMapLabel() {
  $('map-registration').textContent = mapMetadata ? `map5 + 설계 차선 정렬 · ${mapMetadata.width} × ${mapMetadata.height} · ${(mapMetadata.resolution * 100).toFixed(0)} cm/cell` : 'map5 불러오는 중';
}
async function loadMap() {
  try {
    const response = await fetch('/api/map/metadata', {cache:'no-store', signal:AbortSignal.timeout(2500)});
    if (!response.ok) throw new Error('HTTP '+response.status);
    mapMetadata = await response.json();
    $('map5-image').src = '/api/map/image.png';
    setMapLabel();
  } catch (_) { $('map-registration').textContent = 'map5 메타데이터 미수신'; }
}
function renderMapMarkers(robots) {
  const container = $('map-markers'); container.replaceChildren();
  if (!mapMetadata) return;
  robots.forEach((robot, index) => {
    const state = robot.state.data;
    const mapMatches = state?.map_known === true && state.map_width === mapMetadata.width && state.map_height === mapMetadata.height && Math.abs(state.map_resolution - mapMetadata.resolution) < 1e-6 && Math.abs(state.map_origin_x - mapMetadata.origin[0]) < 1e-6 && Math.abs(state.map_origin_y - mapMetadata.origin[1]) < 1e-6;
    if (robot.state.status !== 'live' || state?.localized !== true || state?.header?.frame_id !== 'map' || !mapMatches) return;
    const x = Number(state.x), y = Number(state.y);
    if (!Number.isFinite(x) || !Number.isFinite(y)) return;
    const px = (x - mapMetadata.origin[0]) / mapMetadata.world_width * 100;
    const py = 100 - (y - mapMetadata.origin[1]) / mapMetadata.world_height * 100;
    if (px < 0 || px > 100 || py < 0 || py > 100) return;
    const overhead=robot.overhead.data;
    if (robot.overhead.status === 'live' && overhead?.header?.frame_id === 'map' && Number.isFinite(overhead.position?.x) && Number.isFinite(overhead.position?.y)) {
      const ox=(overhead.position.x-mapMetadata.origin[0])/mapMetadata.world_width*100, oy=100-(overhead.position.y-mapMetadata.origin[1])/mapMetadata.world_height*100;
      if (ox >= 0 && ox <= 100 && oy >= 0 && oy <= 100) {
        const observed=document.createElement('span'); observed.className=`overhead-marker observed-${index+1}`; observed.style.left=`${ox}%`; observed.style.top=`${oy}%`; observed.textContent=`P${index+1}′`; container.append(observed);
      }
    }
    const marker = document.createElement('span');
    marker.className = `map-marker marker-${index + 1}`;
    marker.style.left = `${px}%`; marker.style.top = `${py}%`;
    marker.title = `${robot.name}: map x ${x.toFixed(2)} m, y ${y.toFixed(2)} m`;
    marker.textContent = `P${index + 1}`; container.append(marker);
    const covariance = robot.amcl.data?.covariance;
    if (robot.amcl.status === 'live' && robot.amcl.data?.header?.frame_id === 'map' && Array.isArray(covariance)) {
      const xx=Number(covariance[0]), xy=Number(covariance[1]), yy=Number(covariance[7]);
      const trace=xx+yy, radius=Math.hypot(xx-yy,2*xy), major=(trace+radius)/2, minor=(trace-radius)/2;
      if ([major,minor].every(Number.isFinite) && major >= 0 && minor >= 0) {
        const ellipse=document.createElement('span');
        const angle=-Math.atan2(2*xy,xx-yy)/2*180/Math.PI;
        ellipse.className=`covariance-ellipse ellipse-${index + 1}`;
        ellipse.style.left=`${px}%`; ellipse.style.top=`${py}%`;
        ellipse.style.width=`${4*Math.sqrt(major)/mapMetadata.world_width*100}%`;
        ellipse.style.height=`${4*Math.sqrt(minor)/mapMetadata.world_height*100}%`;
        ellipse.style.transform=`translate(-50%,-50%) rotate(${angle}deg)`;
        ellipse.title=`${robot.name} AMCL 2σ covariance`;
        container.append(ellipse);
      }
    }
    if (state.goal_valid === true && Number.isFinite(state.goal_x) && Number.isFinite(state.goal_y)) {
      const gx=(state.goal_x-mapMetadata.origin[0])/mapMetadata.world_width*100;
      const gy=100-(state.goal_y-mapMetadata.origin[1])/mapMetadata.world_height*100;
      if (gx >= 0 && gx <= 100 && gy >= 0 && gy <= 100) {
        const goal=document.createElement('span'); goal.className='map-goal'; goal.style.left=`${gx}%`; goal.style.top=`${gy}%`;
        goal.textContent='G'; goal.title=`${robot.name} goal: map x ${state.goal_x.toFixed(2)} m, y ${state.goal_y.toFixed(2)} m`; container.append(goal);
      }
    }
  });
}
function ensureCards(robots) {
  const names=JSON.stringify(robots.map(r=>r.name));
  if(names === robotNames) return;
  robotNames=names;
  $('robots').replaceChildren(...robots.map((robot,index)=>{
    const card=$('robot-template').content.firstElementChild.cloneNode(true);
    card.querySelector('h2').textContent=/^pinky\d+$/.test(robot.name) ? robot.name.replace('pinky','Pinky ') : robot.name;
    card.querySelector('.amcl-label').textContent=`P${index+1}: AMCL`;
    card.querySelector('.overhead-label').textContent=`P${index+1}′: Overhead`;
    return card;
  }));
}
function render(data) {
  setControlEnabled(data.controls_enabled === true);
  $('connection').textContent={online:'SYSTEM ONLINE',waiting:'WAITING FOR ROBOTS',degraded:'SYSTEM DEGRADED'}[data.health] || 'UNKNOWN';
  $('connection').className='badge '+(data.health==='online'?'live':'');
  $('connection').title='서버 연결됨 · '+(data.health==='online'?'필수 ROS 상태 수신 중':'로봇 또는 미션 상태 미수신/지연');
  $('mission').textContent=data.mission.status==='live' ? data.mission.data.mission : 'WAITING DATA';
  $('mission').title=age(data.mission);
  $('mission-age').textContent=`미션 · ${age(data.mission)}`;
  $('warning').textContent=data.mission.data?.warning || '';
  const compatibility=Object.entries(data.map_compatibility || {});
  const incompatible=compatibility.filter(([, status]) => status !== 'compatible');
  if (incompatible.length) {
    $('map-registration').textContent=incompatible.map(([name, status]) => `${name}: ${status}`).join(' · ');
  }
  Object.entries(data.cameras || {}).forEach(([name, camera]) => {
    const image=$(name+'-camera-image'), status=$(name+'-camera-status');
    if (!image || !status) return;
    status.textContent=camera.status === 'live' ? `LIVE · ${camera.age_seconds.toFixed(1)}초 전` : camera.status === 'stale' ? 'STALE FEED' : 'WAITING FEED';
    image.classList.toggle('visible', camera.status === 'live');
    if (camera.status === 'live') image.src=`/api/camera/${encodeURIComponent(name)}.jpg?t=${Date.now()}`;
  });
  const observed=data.robots.filter(robot => robot.overhead.status === 'live' && robot.overhead.data?.position);
  const overheadCamera=data.cameras?.overhead;
  $('overhead-system-status').textContent=overheadCamera?.status === 'live' ? `LIVE · ${observed.length} tracked` : observed.length ? `POSE ONLY · ${observed.length} tracked` : 'NOT CONNECTED';
  $('overhead-pose').textContent=observed.length ? observed.map(robot => `P${robot.name.replace('pinky','')}′ x ${number(robot.overhead.data.position.x,'m')} y ${number(robot.overhead.data.position.y,'m')}`).join(' · ') : 'P′ 위치 미수신';
  ensureCards(data.robots);
  renderMapMarkers(data.robots);
  data.robots.forEach((robot,index)=>{
    const card=$('robots').children[index], state=robot.state.data, lane=robot.lane.data, amcl=robot.amcl.data;
    const badge=card.querySelector('.robot-status');
    badge.textContent=robot.lane.status==='live' ? labels[lane.drive_state] || 'UNKNOWN' : 'WAITING DATA';
    badge.className='badge robot-status '+(robot.lane.status==='live'?'live':robot.lane.status==='stale'?'stale':'');
    badge.title=lane?.state_reason || age(robot.lane);
    const laneInfo=$(robot.name+'-lane-info');
    if (laneInfo) {
      if (robot.lane.status !== 'live') laneInfo.textContent='Lane data —';
      else {
        const error=Number.isFinite(lane.error_x_norm) ? `${(lane.error_x_norm * 100).toFixed(0)}%` : '—';
        const hazard=lane.drive_state === 5 ? ` · 장애물 ${number(lane.lidar_min_range,'m')}` : lane.drive_state === 3 || lane.drive_state === 4 ? ' · 횡단보도' : '';
        laneInfo.textContent=`${qualityLabels[lane.lane_quality] || '차선 상태 —'} · 중심 ${error}${hazard}`;
        laneInfo.title=lane.state_reason || `path age ${number(lane.path_age,'s')}`;
      }
    }
    const amclRow=card.querySelector('.pose-row');
    const amclPose=amcl?.position && amcl?.orientation;
    const yaw=amclPose ? Math.atan2(2*(amcl.orientation.w*amcl.orientation.z + amcl.orientation.x*amcl.orientation.y), 1-2*(amcl.orientation.y**2 + amcl.orientation.z**2)) : null;
    const sigma=index => amcl?.covariance && Number.isFinite(amcl.covariance[index]) && amcl.covariance[index] >= 0 ? Math.sqrt(amcl.covariance[index]) : null;
    amclRow.querySelector('.pose-value').textContent=amclPose ? `x ${number(amcl.position.x,'m')}  y ${number(amcl.position.y,'m')}  yaw ${number(yaw*180/Math.PI,'°',1)}` : 'x —   y —   yaw —';
    amclRow.querySelector('.covariance').textContent=amcl ? `σx ${number(sigma(0),'m',3)}  σy ${number(sigma(7),'m',3)}  σyaw ${number(sigma(35)*180/Math.PI,'°',2)}` : 'σx —   σy —   σyaw —';
    amclRow.querySelector('.data-status').textContent=age(robot.amcl);
    amclRow.title=amcl?.header?.frame_id ? `AMCL frame ${amcl.header.frame_id} · ${age(robot.amcl)}` : age(robot.amcl);
    const overheadRow=card.querySelector('.overhead-row'), overhead=robot.overhead.data;
    const overheadPose=overhead?.position && overhead?.orientation;
    const overheadYaw=overheadPose ? Math.atan2(2*(overhead.orientation.w*overhead.orientation.z + overhead.orientation.x*overhead.orientation.y), 1-2*(overhead.orientation.y**2 + overhead.orientation.z**2)) : null;
    overheadRow.querySelector('.pose-value').textContent=overheadPose ? `x ${number(overhead.position.x,'m')}  y ${number(overhead.position.y,'m')}  yaw ${number(overheadYaw*180/Math.PI,'°',1)}` : 'x —   y —   yaw —';
    overheadRow.querySelector('.data-status').textContent=age(robot.overhead);
    const pose=state?.localized===true;
    card.querySelector('.tf-value').textContent=pose ? `ROS 위치(TF) x ${number(state.x,'m')}  y ${number(state.y,'m')}  yaw ${number(state.yaw*180/Math.PI,'°',1)}` : 'ROS 위치(TF) —';
    card.querySelector('.actual-speed').textContent=`실제 속도 ${number(state?.linear_velocity,'m/s')} · ${number(state?.angular_velocity,'rad/s')}`;
    card.querySelector('.live-detail').title=`${age(robot.state)} · frame ${state?.header?.frame_id || '—'}`;
    const linearInput=card.querySelector('.linear-limit'), angularInput=card.querySelector('.angular-limit');
    if (document.activeElement !== linearInput) linearInput.value=state ? number(state.max_linear_vel,'') : '';
    if (document.activeElement !== angularInput) angularInput.value=state ? number(state.max_angular_vel,'') : '';
    card.querySelector('.robot-age').textContent=age(robot.state);
    card.querySelector('.apply').onclick=async () => {
      try { await control({action:'speed', robot:robot.name, linear:Number(card.querySelector('.linear-limit').value), angular:Number(card.querySelector('.angular-limit').value)}); } catch (_) { $('warning').textContent='속도 변경 요청이 거부되었습니다'; }
    };
    card.querySelector('.tf-value').classList.toggle('stale-text',robot.state.status==='stale');
  });
}
async function poll() {
  try {
    const response=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(2500)});
    if(!response.ok) throw new Error('HTTP '+response.status);
    render(await response.json()); $('robots').classList.remove('disconnected');
  } catch(error) {
    $('connection').textContent='SERVER DISCONNECTED'; $('connection').className='badge stale';
    $('connection').title='서버 연결 끊김 · 표시값은 마지막 수신값';
    $('mission').textContent='UNKNOWN'; $('mission-age').textContent='서버 연결 끊김 · 표시값은 마지막 수신값';
    $('robots').classList.add('disconnected');
    [...$('robots').children].forEach(card=>{card.querySelector('.robot-status').textContent='UNKNOWN';card.querySelector('.robot-status').className='badge robot-status stale';});
  } finally { setTimeout(poll,500); }
}
ensureCards([{name:'pinky1'},{name:'pinky2'}]);
['start','pause','resume','reset'].forEach(action => $(action).addEventListener('click', async () => { try { if (action === 'reset') $('control-status').textContent='수동 배치 좌표로 AMCL 초기화 요청 중'; await control({action}); } catch (_) { $('warning').textContent='제어 요청이 거부되었습니다'; } }));
loadMap();
poll();
