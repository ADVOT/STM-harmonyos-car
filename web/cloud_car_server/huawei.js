'use strict';

/*
 * 华为云 IoTDA 应用侧调用 —— 使用官方 SDK（@huaweicloud/huaweicloud-sdk-iotda）
 * 签名(SDK-HMAC-SHA256 / 派生 iotdm)由 SDK 内部处理，无需手写。
 * 对外暴露：sendCommand(dir,duration)、getShadow()、getDeviceStatus()，均返回归一化结果
 *   { ok, status, data?, error_code?, error_msg? }
 */

const iotda = require('@huaweicloud/huaweicloud-sdk-iotda');
const core = require('@huaweicloud/huaweicloud-sdk-core');

const AK = process.env.HW_AK;
const SK = process.env.HW_SK;
const PROJECT_ID = process.env.HW_PROJECT_ID;
const DEVICE_ID = process.env.HW_DEVICE_ID;
const ENDPOINT = process.env.HW_IOTDA_APP_ENDPOINT;
const SERVICE_ID = process.env.HW_SERVICE_ID || 'Industrial_Internet';

function buildClient() {
  if (!AK || !SK || !PROJECT_ID || !DEVICE_ID || !ENDPOINT) {
    throw new Error('缺少配置：请在 .env 填写 HW_AK/HW_SK/HW_PROJECT_ID/HW_DEVICE_ID/HW_IOTDA_APP_ENDPOINT');
  }
  const cred = new core.BasicCredentials({ ak: AK, sk: SK, projectId: PROJECT_ID });
  const host = String(ENDPOINT).replace(/^https?:\/\//, '').replace(/\/+$/, '');
  const regionId = process.env.HW_REGION || 'cn-north-4';
  // 标准版实例应用侧接入地址是实例专属域名(xxx.st1.iotda-app.region.myhuaweicloud.com)。
  // 该专属域名要求【派生签名 V11-HMAC-SHA256(service=iotdm, region)】，
  // 但 SDK 默认不设置 derivedPredicate -> 会退回标准 SDK-HMAC-SHA256 签名 -> 专属网关拒签 401 IOTDA.000002。
  // getDefaultDerivedPredicate 按域名判定：专属域名(不匹配默认正则)用派生签名，通用域名用标准签名。
  // 派生 info 需要 region，故同时用 new Region(regionId, 专属域名) 提供 region.id 与 endpoint。
  cred.withDerivedPredicate(
    core.BasicCredentials.getDefaultDerivedPredicate.bind(core.BasicCredentials)
  );
  const region = new core.Region(regionId, 'https://' + host);
  return iotda.IoTDAClient.newBuilder()
    .withCredential(cred)
    .withRegion(region)
    .build();
}

// 把 SDK 抛出的错误归一化
// 兼容两种形态：axios 风格 e.response.{status,data}；
// 以及官方 SDK 的 ClientRequestException（顶层 httpStatusCode/errorCode/errorMsg）。
function normalizeError(e) {
  const r = (e && e.response) || {};
  const body = r.data || (e && e.data) || {};
  const status = r.status || (e && e.httpStatusCode) || (e && e.status) || (e && e.statusCode);
  const code = body.error_code || (e && e.errorCode) || (e && e.error_code);
  let msg = body.error_msg || (e && e.errorMsg) || (e && e.error_msg) || (e && e.message);
  if (!status && !code && !msg) {
    try { msg = JSON.stringify(e, Object.getOwnPropertyNames(e)); } catch (_) { msg = String(e); }
  }
  return { ok: false, status, error_code: code, error_msg: msg };
}

// 命令下发：move {dir, duration}
async function sendCommand(dir, duration) {
  try {
    const client = buildClient();
    const body = new iotda.DeviceCommandRequest();
    body.serviceId = SERVICE_ID;
    body.commandName = 'move';
    body.paras = { dir, duration };
    const req = new iotda.CreateCommandRequest(DEVICE_ID);
    req.withBody(body);
    const resp = await client.createCommand(req);
    return { ok: true, status: 200, data: resp };
  } catch (e) {
    return normalizeError(e);
  }
}

// 查设备影子（阶段二遥测回显）
async function getShadow() {
  try {
    const client = buildClient();
    const req = new iotda.ShowDeviceShadowRequest(DEVICE_ID);
    const resp = await client.showDeviceShadow(req);
    return { ok: true, status: 200, data: resp };
  } catch (e) {
    return normalizeError(e);
  }
}

// 查设备真实在线状态（ONLINE/OFFLINE）与最近连接更新时间
async function getDeviceStatus() {
  try {
    const client = buildClient();
    const req = new iotda.ShowDeviceRequest(DEVICE_ID);
    const resp = await client.showDevice(req);
    return {
      ok: true,
      status: 200,
      data: {
        status: resp.status || null,
        connection_update_time: resp.connectionStatusUpdateTime || resp.connection_status_update_time || null,
        device_name: resp.deviceName || resp.device_name || null,
      },
    };
  } catch (e) {
    return normalizeError(e);
  }
}

module.exports = { sendCommand, getShadow, getDeviceStatus };
