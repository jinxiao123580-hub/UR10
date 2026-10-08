<# 拉取本套文档用到的配图到 docs/assets/
用法：在联网环境执行
  powershell -ExecutionPolicy Bypass -File "docs/scripts/fetch_images.ps1"
产出：assets/ 下两组图
  - Wikimedia 开放授权图（PD / CC）
  - 开源教程 README 图（供实验室内部学习·非商用；出处见 docs/06）
出处与授权见 docs/06-入门-图片与授权清单.md
#>
$ErrorActionPreference = 'Stop'
$dir = Join-Path $PSScriptRoot '..\assets'
New-Item -ItemType Directory -Force -Path $dir | Out-Null
$ua = 'UR10-Docs/1.0'

# 第一组：Wikimedia（开放授权）
$wiki = [ordered]@{
  'pinhole-camera.svg'    = 'https://upload.wikimedia.org/wikipedia/commons/3/3b/Pinhole-camera.svg'
  'ur-dh-diagram.png'     = 'https://upload.wikimedia.org/wikipedia/commons/3/3f/Sample_Denavit-Hartenberg_Diagram.png'
  'cobot-demo.jpg'        = 'https://upload.wikimedia.org/wikipedia/commons/b/ba/Japan-Mobility-Show-2023-RuinDig_586.jpg'
  '6dof.svg'              = 'https://upload.wikimedia.org/wikipedia/commons/2/2a/6DOF.svg'
  'eulerangles.svg'       = 'https://upload.wikimedia.org/wikipedia/commons/a/a1/Eulerangles.svg'
  'fanuc-robot.jpg'       = 'https://upload.wikimedia.org/wikipedia/commons/e/e4/FANUC_welding_robot_reaching.jpg'
  'fwd-inv-kinematics.png'= 'https://upload.wikimedia.org/wikipedia/commons/f/f4/FWDvsINV_Kinematics_HighResTransp.png'
  'robot-arm-model.png'   = 'https://upload.wikimedia.org/wikipedia/commons/9/9b/Robot_arm_model_1.png'
  'center-of-mass.svg'    = 'https://upload.wikimedia.org/wikipedia/commons/b/bc/Center_of_mass_two_bodies.svg'
  'end-effector-shadow-hand.png' = 'https://upload.wikimedia.org/wikipedia/commons/f/f4/Shadow_Hand_Bulb_large_Alpha.png'
  'cmm-machine.png'   = 'https://upload.wikimedia.org/wikipedia/commons/b/bf/9.12.17_Coordinate_measuring_machine.png'
}

# 第二组：开源教程 README 图（非商用·内部学习）
$R = 'https://raw.githubusercontent.com/RealManRobot/hand_eye_calibration/master/picture/'
$open = [ordered]@{
  'ft_gravity_model.png'  = 'https://raw.githubusercontent.com/GeneHit/hand_force_calibration/master/modeling.png'
  'handeye_eye_in_hand.jpg' = ($R + 'f6c716fb-c8d2-4adc-b3da-a86c6b1e78d0.png')
  'handeye_eye_to_hand.png' = ($R + '44776e79-47f7-4de2-9ef2-172b654169d5.png')
  'handeye_loop_axxb.png' = ($R + '29fb4d433468f12530eca3e2a563da72.png')
  'handeye_calib_plate.png' = 'https://raw.githubusercontent.com/pyni/handeye_calibration_with_depth_camera/master/figure/block.png'
  'calibboard.png'        = ($R + '%E6%A0%87%E5%AE%9A%E6%9D%BF%E5%9B%BE%E7%89%87_00(1).png')
}

foreach ($m in @($wiki, $open)) {
  foreach ($k in $m.Keys) {
    Invoke-WebRequest -Uri $m[$k] -OutFile (Join-Path $dir $k) -UserAgent $ua -TimeoutSec 60
    Write-Host "OK  $k  ->  docs/assets/$k"
  }
}
Write-Host "完成。请在文档里把对应引用改成 assets/<文件名>（热链接图保持 https 直链即可）"