export class DashboardApi {
  async fetchState() {
    const response = await fetch('/api/state', { cache: 'no-store' });
    return response.json();
  }
}
