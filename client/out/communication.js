var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
export class Communication {
    constructor(address, port) {
        this.address = address;
        this.port = port;
    }
    get(route) {
        return __awaiter(this, void 0, void 0, function* () {
            return yield (yield fetch(`http://${this.address}:${this.port}/${route}`)).json();
        });
    }
    post(route, body) {
        return __awaiter(this, void 0, void 0, function* () {
            return yield (yield fetch(`http://${this.address}:${this.port}/${route}`, { method: "POST", body: JSON.stringify(body) })).json();
        });
    }
}
