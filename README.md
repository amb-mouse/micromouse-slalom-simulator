# Micromouse Slalom Simulator

Cloudflare Workers + Static Assets 用の静的Webシミュレータです。

## 内容

- `motion_speed_profiles` の 200 / 300 / 400 / 500 mm/s プロファイル
- `turn_sim_pygame.py` を元にした 1 ms 刻みのターン計算
- 角加速 → 等角速度 → 角減速
- スリップ角係数による進行方向補正
- 左 / 右 / 左→右 / 右→左
- 左→右・右→左の2個目入口追加補正
- 682 g / トレッド81 mm / タイヤ幅6 mm
- 横加速度、横G、横力、必要摩擦係数の表示

## ローカル起動

```bash
npm install
npm run dev
```

## Cloudflareへ公開

```bash
npm install
npx wrangler login
npm run deploy
```
